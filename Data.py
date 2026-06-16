import serial
import wave
import time
from pathlib import Path

BAUD = 460800
PORT = "COM4"

# ESP -> Python
OUT_STATE    = 0x01
OUT_AUDIO    = 0x02
OUT_PLAY_ACK = 0x03

# Python -> ESP
IN_PLAY_START = 0x10
IN_PLAY_AUDIO = 0x11
IN_PLAY_END   = 0x12

SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2
PLAY_CHUNK_SAMPLES = 256

record_file = None
record_filename = None


def pkt_write(ser, pkt_type, payload=b""):
    pkt_len = len(payload)
    hdr = bytes([0xFF, 0xFE, pkt_type, (pkt_len >> 8) & 0xFF, pkt_len & 0xFF])
    ser.write(hdr)
    if pkt_len:
        ser.write(payload)


def read_exact(ser, n):
    data = bytearray()
    while len(data) < n:
        chunk = ser.read(n - len(data))
        if not chunk:
            raise serial.SerialException("Serial timeout / disconnect")
        data.extend(chunk)
    return bytes(data)


def read_packet(ser):
    while True:
        if read_exact(ser, 1) != b'\xff':
            continue
        if read_exact(ser, 1) != b'\xfe':
            continue

        hdr = read_exact(ser, 3)
        pkt_type = hdr[0]
        pkt_len = (hdr[1] << 8) | hdr[2]
        payload = read_exact(ser, pkt_len) if pkt_len else b""
        return pkt_type, payload


def wait_for_ack(ser, expect_ack=True):
    while True:
        pkt_type, payload = read_packet(ser)

        if pkt_type == OUT_PLAY_ACK:
            return True

        if pkt_type == OUT_STATE:
            state = payload.decode(errors="ignore")
            print("[ESP STATE]", state)
            # state paketlerini yut, ACK beklemeye devam et
            continue

        # Playback sırasında OUT_AUDIO beklemiyoruz; gelirse loglayıp devam et
        if pkt_type == OUT_AUDIO:
            print("[PY] Unexpected OUT_AUDIO during ACK wait, ignored.")
            continue


def start_recording_file():
    global record_file, record_filename

    folder = Path(".")
    record_filename = folder / f"record_{int(time.time())}.wav"

    wf = wave.open(str(record_filename), "wb")
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(SAMPLE_WIDTH)
    wf.setframerate(SAMPLE_RATE)

    record_file = wf
    print(f"[PY] Recording to: {record_filename}")


def close_recording_file():
    global record_file, record_filename

    if record_file is not None:
        record_file.close()
        print(f"[PY] WAV saved: {record_filename}")
        record_file = None

    return record_filename


def play_wav_back(ser, filename):
    if filename is None:
        print("[PY] No WAV file to play back.")
        return

    with wave.open(str(filename), "rb") as wf:
        ch = wf.getnchannels()
        sw = wf.getsampwidth()
        sr = wf.getframerate()

        if ch != CHANNELS or sw != SAMPLE_WIDTH or sr != SAMPLE_RATE:
            raise ValueError(
                f"WAV format mismatch. Needed mono/16-bit/16000, got ch={ch}, sw={sw}, sr={sr}"
            )

        print(f"[PY] Sending WAV back to ESP: {filename}")

        pkt_write(ser, IN_PLAY_START)
        wait_for_ack(ser)

        while True:
            frames = wf.readframes(PLAY_CHUNK_SAMPLES)
            if not frames:
                break

            pkt_write(ser, IN_PLAY_AUDIO, frames)
            wait_for_ack(ser)

        pkt_write(ser, IN_PLAY_END)
        wait_for_ack(ser)

        print("[PY] Playback stream finished.")


def main():
    global record_file

    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=2,
        write_timeout=2
    )
    time.sleep(2)

    print("[PY] Listening...")

    try:
        while True:
            pkt_type, payload = read_packet(ser)

            if pkt_type == OUT_STATE:
                state = payload.decode(errors="ignore")
                print("[ESP STATE]", state)

                if state == "RECORDING":
                    if record_file is None:
                        start_recording_file()

                elif state == "STOP":
                    filename = close_recording_file()
                    time.sleep(0.1)
                    play_wav_back(ser, filename)

            elif pkt_type == OUT_AUDIO:
                if record_file is not None:
                    record_file.writeframesraw(payload)

            elif pkt_type == OUT_PLAY_ACK:
                # Normal ana döngüde gelirse sadece logla
                print("[PY] Unexpected ACK in main loop, ignored.")

    except KeyboardInterrupt:
        print("\n[PY] Stopped by user.")


    finally:
        if record_file is not None:
            record_file.close()
        ser.close()


if __name__ == "__main__":
    main()