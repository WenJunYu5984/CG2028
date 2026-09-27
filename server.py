import os
import smtplib
import socket
import socketserver
import ssl
import threading
import time

from email.message import EmailMessage


HOST = "0.0.0.0"
PORT = 2028

EXPECTED_PACKET = b"FallDetected"
MAX_PACKET = 64

GMAIL_USER = "xuzihao024572@gmail.com"
GMAIL_PASSWORD = os.environ.get("GMAIL_APP_PASSWORD")

if not GMAIL_PASSWORD:
    raise RuntimeError("GMAIL_APP_PASSWORD environment variable is not set")

RECIPIENTS = [
    "xuzihao024572@gmail.com",
    "wenjunyu2468@gmail.com",
]

if not RECIPIENTS:
    raise RuntimeError("At least one recipient must be configured")

EMAIL_SUBJECT = "CG2028 Project: We detected a fall"

EMAIL_BODY = (
    "Hi,\n\n"
    "Our wearable device has detected a fall for our user.\n"
)
EMAIL_COOLDOWN_SECONDS = 30

last_email_attempt = 0.0
email_cooldown_lock = threading.Lock()


def reserve_email_attempt():
    global last_email_attempt

    with email_cooldown_lock:
        now = time.monotonic()

        if now - last_email_attempt < EMAIL_COOLDOWN_SECONDS:
            return False

        last_email_attempt = now
        return True


def send_fall_email():
    if not reserve_email_attempt():
        print("Fall email suppressed by cooldown.")
        return "EMAIL_COOLDOWN"

    message = EmailMessage()
    message["From"] = GMAIL_USER

    message["To"] = "undisclosed-recipients:;"
    message["Subject"] = EMAIL_SUBJECT
    message.set_content(EMAIL_BODY)

    tls_context = ssl.create_default_context()

    try:
        with smtplib.SMTP_SSL(
            host="smtp.gmail.com",
            port=465,
            context=tls_context,
            timeout=10,
        ) as smtp:
            smtp.login(GMAIL_USER, GMAIL_PASSWORD)

            refused = smtp.send_message(
                message,
                from_addr=GMAIL_USER,
                to_addrs=RECIPIENTS,
            )

        if refused:
            print(f"Some recipients were rejected: {refused}")
            return "EMAIL_PARTIAL"

        print("Fall email accepted for all recipients.")
        return "EMAIL_SENT"

    except smtplib.SMTPException as error:
        print(f"SMTP operation failed: {error}")
        return "EMAIL_FAILED"

    except OSError as error:
        print(f"Network connection to Gmail failed: {error}")
        return "EMAIL_FAILED"


class RequestHandler(socketserver.BaseRequestHandler):
    def handle(self):
        print(f"Connected: {self.client_address}")
        self.request.settimeout(600)
        buffer = b""

        try:
            while True:
                data = self.request.recv(128)

                if not data:
                    break

                buffer += data

                while b"\r" in buffer:
                    packet, buffer = buffer.split(b"\r", 1)

                    if len(packet) > MAX_PACKET:
                        self.request.sendall(b"ERR_PACKET_TOO_LARGE\r")
                        return

                    if packet != EXPECTED_PACKET:
                        # Do not echo or process unknown input.
                        self.request.sendall(b"IGNORED\r")
                        continue

                    print(f"Fall detected by {self.client_address}")

                    result = send_fall_email()
                    self.request.sendall(result.encode("ascii") + b"\r")

                if len(buffer) > MAX_PACKET:
                    self.request.sendall(b"ERR_PACKET_TOO_LARGE\r")
                    return

        except socket.timeout:
            print(f"Connection timed out: {self.client_address}")

        except ConnectionError as error:
            print(f"Connection lost: {self.client_address}: {error}")

        finally:
            print(f"Disconnected: {self.client_address}")


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True
    request_queue_size = 10


def main():
    with Server((HOST, PORT), RequestHandler) as server:
        print(f"Fall-detection server listening on port {PORT}")
        server.serve_forever()


if __name__ == "__main__":
    main()
