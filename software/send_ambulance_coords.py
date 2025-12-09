# small test script to send coordinates to the drone listener
import socket, json
def send_coords(host='127.0.0.1', port=9000, lat=23.8, lon=86.4):
    payload = {'lat': lat, 'lon': lon, 'id': 'ambulance1'}
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    s.send(json.dumps(payload).encode('utf-8'))
    s.close()

if __name__ == '__main__':
    send_coords()
    print("Sent coords to listener")
