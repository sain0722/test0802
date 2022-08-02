import json
import socket

class Server:
    def __init__(self):
        # 접속할 서버 주소입니다. 여기에서는 루프백(loopback) 인터페이스 주소 즉 localhost를 사용합니다.
        self.HOST = '152.69.224.89'
        # 클라이언트 접속을 대기하는 포트 번호입니다.
        self.PORT = 9999

        # 소켓 객체를 생성합니다.
        # 주소 체계(address family)로 IPv4, 소켓 타입으로 TCP 사용합니다.
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # 포트 사용중이라 연결할 수 없다는
        # WinError 10048 에러 해결를 위해 필요합니다.
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # bind 함수는 소켓을 특정 네트워크 인터페이스와 포트 번호에 연결하는데 사용됩니다.
        # HOST는 hostname, ip address, 빈 문자열 ""이 될 수 있습니다.
        # 빈 문자열이면 모든 네트워크 인터페이스로부터의 접속을 허용합니다.
        # PORT는 1-65535 사이의 숫자를 사용할 수 있습니다.
        self.server_socket.bind((self.HOST, self.PORT))

        # 서버가 클라이언트의 접속을 허용하도록 합니다.
        self.server_socket.listen()

        self.client_socket = None
        self.addr = None

    def __del__(self):
        self.server_socket.close()

    def run(self):
        self.wait_connect()

    def wait_connect(self):
        # accept 함수에서 대기하다가 클라이언트가 접속하면 새로운 소켓을 리턴합니다.
        self.client_socket, self.addr = self.server_socket.accept()
        # print(self.client_socket, self.addr)

        # 접속한 클라이언트의 주소입니다.
        print('Connected by', self.addr)
        self.connect_client()

    # 무한루프를 돌면서
    def connect_client(self):
        while True:

            # 클라이언트가 보낸 메시지를 수신하기 위해 대기합니다.
            data = self.client_socket.recv(1024)

            # if not data:
            #     break
            if data:
                decoded_data = data.decode('utf-8')
                # print(data)
                input_id, input_pw = decoded_data.split()
                # input_dictionary = {
                #     inputid: inputpw
                # }
                print(input_id, input_pw)
                with open("Authentication.json") as f:
                    json_data = json.load(f)

                    for name, password in json_data.items():
                        if name == input_id:
                            if password == input_pw:
                                print("일치함")
                                result = 'OK'
                            else:
                                result = 'NG'
                            send_result = result.encode()
                            self.client_socket.sendall(send_result)
                break
        self.client_socket.close()
        self.wait_connect()


def main():
    server = Server()
    server.run()

    # print(jsondata)
    # # 빈 문자열을 수신하면 루프를 중지합니다.
    # if not data:
    #     break
    #
    # # 수신받은 문자열을 출력합니다.
    # print('Received from', addr, data.decode())
    #
    # # 받은 문자열을 다시 클라이언트로 전송해줍니다.(에코)
    # client_socket.sendall(data)
    # client_socket.close()


if __name__ == "__main__":
    main()