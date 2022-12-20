import socket
import time


def main(port: int = 1234):
    host = "127.0.0.1"
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))

    count = 0

    while True:
        time.sleep(0.5)
        print(count)

        sock.sendall(str(count).encode("UTF-8"))

        received_data = sock.recv(1024).decode("UTF-8")
        print(received_data)
        if received_data is not None:
            count += 1


if __name__ == '__main__':
    main(1234)
