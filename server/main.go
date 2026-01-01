package main

import (
        "bytes"
        "encoding/json"
        "fmt"
        "log"
        "net/http"
        "os"
        "sync"
        "time"
        "unsafe"

        "golang.org/x/sys/unix"
        "periph.io/x/conn/v3/gpio"
        "periph.io/x/conn/v3/gpio/gpioreg"
        "periph.io/x/host/v3"
)

const (
        FLOW_ADDR           = 0x55
        SERIAL_TX_MAX_FRAME = 256
)

// --- CRC8 table (poly=0x07) ---
var crc8Table = []byte{
        0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
        0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
        0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
        0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
        0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,
        0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
        0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
        0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
        0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
        0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
        0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,
        0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
        0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,
        0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
        0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
        0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
        0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,
        0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
        0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
        0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
        0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
        0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
        0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,
        0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
        0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,
        0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
        0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
        0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
        0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,
        0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
        0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,
        0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3,
}

func crc8(data []byte) byte {
        crc := byte(0)
        for _, b := range data {
                crc = crc8Table[crc^b]
        }
        return crc
}

// SendFrame sends a frame over serial
func SendFrame(f *os.File, payload []byte) error {
        if len(payload) > SERIAL_TX_MAX_FRAME {
                return os.ErrInvalid
        }
        frame := []byte{FLOW_ADDR, byte(len(payload) >> 8), byte(len(payload) & 0xFF)}
        frame = append(frame, payload...)
        frame = append(frame, crc8(frame))
        _, err := f.Write(frame)
        if err != nil {
                return err
        }
        unix.IoctlSetInt(int(f.Fd()), unix.TCSBRK, 1)
        return nil
}

// --- Payloads ---
type FlowPayload struct {
        Flow      float64 `json:"flow"`
        ShowerNum int     `json:"shower_num"`
}

type FlowTempPayload struct {
        Flow      float64 `json:"flow"`
        Temp      float64 `json:"temp"`
        ShowerNum int     `json:"shower_num"`
}

// --- Global lock for MCU communication ---
var serialLock sync.Mutex

// ReadMCUFrames reads **all valid frames** for FLOW_ADDR within timeout
func ReadMCUFrames(f *os.File, expectedAddr byte, expectedContent []byte, timeoutMs int) ([]byte, error) {
        buf := make([]byte, 1024)
        pos := 0
        deadline := time.Now().Add(time.Duration(timeoutMs) * time.Millisecond)
        for {
                if time.Now().After(deadline) {
                        return nil, fmt.Errorf("timeout waiting for MCU frames")
                }
                n, err := unix.Read(int(f.Fd()), buf[pos:])
                if n > 0 {
                        pos += n
                }
                if err != nil && err != unix.EAGAIN && err != os.ErrNoDeadline {
                        return nil, err
                }

                i := 0
                for i <= pos-4 {
                        addr := buf[i]
                        if addr != expectedAddr {
                                i++
                                continue
                        }
                        length := int(uint16(buf[i+1])<<8 | uint16(buf[i+2]))
                        if i+3+int(length) >= pos {
                                break // incomplete
                        }

                        payload := buf[i+3:i+3+length]
                        crc := buf[i+3+length]
                        crcBuf := buf[i:i+3+length]
                        //log.Println("PAY %s", string(payload))
                        if crc == crc8(crcBuf) && bytes.Contains(payload, expectedContent) {
                                return payload, nil
                        }

                        i += 3 + int(length) + 1
                }

                if i < pos {
                        copy(buf[0:], buf[i:pos])
                        pos = pos - i
                } else {
                        pos = 0
                }

                time.Sleep(5 * time.Millisecond)
        }
}

// sendMCUCommand sends a command and returns **the frame containing keyword** or last frame
func sendMCUCommand(f *os.File, pin gpio.PinOut, frame []byte, keyword []byte, timeoutMs int) ([]byte, error) {
        serialLock.Lock()
        defer serialLock.Unlock()

        if err := pin.Out(gpio.High); err != nil {
                return nil, err
        }
        err := SendFrame(f, frame)
        _ = pin.Out(gpio.Low)
        if err != nil {
                return nil, err
        }

        if len(keyword) != 0 {
                receivedFrame, err := ReadMCUFrames(f, FLOW_ADDR, keyword, timeoutMs)
                if err != nil {
                        return nil, err
                }
                return receivedFrame, nil
        }
        return nil, nil
}

func main() {
        if _, err := host.Init(); err != nil {
                log.Fatal(err)
        }
        pin := gpioreg.ByName("GPIO4")
        if pin == nil {
                log.Fatal("failed to find GPIO4")
        }
        pin.Out(gpio.Low)

        f, err := os.OpenFile("/dev/ttyAMA0", unix.O_RDWR|unix.O_NOCTTY|unix.O_NONBLOCK, 0666)
        if err != nil {
                log.Fatal(err)
        }
        defer f.Close()

        cflag := uint32(unix.CREAD | unix.CLOCAL | unix.B115200 | unix.CS8)
        t := unix.Termios{Iflag: unix.IGNPAR, Cflag: cflag, Ispeed: unix.B115200, Ospeed: unix.B115200}
        t.Cc[unix.VMIN] = 1
        t.Cc[unix.VTIME] = 0
        fd := f.Fd()
        if _, _, errno := unix.Syscall6(unix.SYS_IOCTL, uintptr(fd), uintptr(unix.TCSETS), uintptr(unsafe.Pointer(&t)), 0, 0, 0); errno != 0 {
                log.Fatalf("Errno = %d", errno)
        }
        //unix.SetNonblock(int(fd), true) //

        // --- POST Handlers ---
        http.HandleFunc("/flow/stop", func(w http.ResponseWriter, r *http.Request) {
                if r.Method != http.MethodPost {
                        w.WriteHeader(http.StatusMethodNotAllowed)
                        return
                }
                resp, err := sendMCUCommand(f, pin, []byte{0}, []byte("NOFLOW: STARTED"), 5000)
                if err != nil {
                        w.WriteHeader(http.StatusGatewayTimeout)
                        w.Write([]byte(fmt.Sprintf("MCU response error: %v, last frame: %s", err, resp)))
                        return
                }
                w.Write(resp)
        })

        http.HandleFunc("/flow/cold", func(w http.ResponseWriter, r *http.Request) {
                if r.Method != http.MethodPost {
                        w.WriteHeader(http.StatusMethodNotAllowed)
                        return
                }
                var req FlowPayload
                if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
                        w.WriteHeader(http.StatusBadRequest)
                        return
                }
                flowInt := int(req.Flow)
                frame := []byte{1, byte(flowInt >> 8), byte(flowInt & 0xFF), byte(req.ShowerNum), 0}
                resp, err := sendMCUCommand(f, pin, frame, []byte("COLDFLOW: STARTING"), 3000)
                if err != nil {
                        w.WriteHeader(http.StatusGatewayTimeout)
                        w.Write([]byte(fmt.Sprintf("MCU response error: %v, last frame: %s", err, resp)))
                        return
                }
                w.Write(resp)
        })

        http.HandleFunc("/flow/hot", func(w http.ResponseWriter, r *http.Request) {
                if r.Method != http.MethodPost {
                        w.WriteHeader(http.StatusMethodNotAllowed)
                        return
                }
                var req FlowPayload
                if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
                        w.WriteHeader(http.StatusBadRequest)
                        return
                }
                flowInt := int(req.Flow)
                frame := []byte{2, byte(flowInt >> 8), byte(flowInt & 0xFF), byte(req.ShowerNum), 0}
                resp, err := sendMCUCommand(f, pin, frame, []byte("HOTFLOW: STARTED"), 3000)
                if err != nil {
                        w.WriteHeader(http.StatusGatewayTimeout)
                        w.Write([]byte(fmt.Sprintf("MCU response error: %v, last frame: %s", err, resp)))
                        return
                }
                w.Write(resp)
        })

        http.HandleFunc("/flow/ff", func(w http.ResponseWriter, r *http.Request) {
                if r.Method != http.MethodPost {
                        w.WriteHeader(http.StatusMethodNotAllowed)
                        return
                }
                var req FlowTempPayload
                if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
                        w.WriteHeader(http.StatusBadRequest)
                        return
                }
                flowInt := int(req.Flow)
                tempInt := int(req.Temp * 16)
                frame := []byte{
                        3,
                        byte(flowInt >> 8), byte(flowInt & 0xFF),
                        byte(tempInt >> 8), byte(tempInt & 0xFF),
                        byte(req.ShowerNum),
                }
                resp, err := sendMCUCommand(f, pin, frame, []byte("FFFLOW: STARTED"), 3000)
                if err != nil {
                        w.WriteHeader(http.StatusGatewayTimeout)
                        w.Write([]byte(fmt.Sprintf("MCU response error: %v, last frame: %s", err, resp)))
                        return
                }
                w.Write(resp)
        })

        // --- GET Handlers ---
        http.HandleFunc("/get/pos", func(w http.ResponseWriter, r *http.Request) {
                if r.Method != http.MethodGet {
                        w.WriteHeader(http.StatusMethodNotAllowed)
                        return
                }
                resp, err := sendMCUCommand(f, pin, []byte{10}, []byte("GETPOS: OK"), 500)
                if err != nil {
                        w.WriteHeader(http.StatusGatewayTimeout)
                        w.Write([]byte(fmt.Sprintf("MCU response error: %v, last frame: %s", err, resp)))
                        return
                }
                w.Write(resp)
        })

        http.HandleFunc("/get/flow", func(w http.ResponseWriter, r *http.Request) {
                if r.Method != http.MethodGet {
                        w.WriteHeader(http.StatusMethodNotAllowed)
                        return
                }
                resp, err := sendMCUCommand(f, pin, []byte{11}, []byte("GETFLOW: OK"), 500)
                if err != nil {
                        w.WriteHeader(http.StatusGatewayTimeout)
                        w.Write([]byte(fmt.Sprintf("MCU response error: %v, last frame: %s", err, resp)))
                        return
                }
                w.Write(resp)
        })

        http.HandleFunc("/get/temp", func(w http.ResponseWriter, r *http.Request) {
                if r.Method != http.MethodGet {
                        w.WriteHeader(http.StatusMethodNotAllowed)
                        return
                }
                resp, err := sendMCUCommand(f, pin, []byte{12}, []byte("GETTEMP: OK"), 500)
                if err != nil {
                        w.WriteHeader(http.StatusGatewayTimeout)
                        w.Write([]byte(fmt.Sprintf("MCU response error: %v, last frame: %s", err, resp)))
                        return
                }
                w.Write(resp)
        })

        log.Println("Server listening on :8080")
        log.Fatal(http.ListenAndServe(":8080", nil))
}
