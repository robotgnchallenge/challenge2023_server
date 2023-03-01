def socket_conf():
    HEADER = 64
    PORT = 8080
    FORMAT = 'utf-8'
    DISCONNECT_MSG = "[DISCONNECT SERVICE] ..."
    SERVER = "127.0.0.1"
    ADDR = (SERVER, PORT)
    return ADDR, HEADER, FORMAT, DISCONNECT_MSG
