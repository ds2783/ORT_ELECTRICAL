import socket

def relay_udp_data(listen_ip, listen_port, forward_ip1, forward_port1, forward_ip2, forward_port2):
    # Create a UDP socket to listen for incoming data
    listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listen_sock.bind((listen_ip, listen_port))  # Bind to the listen port
    
    print(f"Listening for incoming data on {listen_ip}:{listen_port}...")

    while True:
        # Receive data from the listening port
        data, addr = listen_sock.recvfrom(1024)  # Buffer size is 1024 bytes

        if data:
            # Forward the received data to the two other ports
            # Send data to the first destination
            listen_sock.sendto(data, (forward_ip1, forward_port1))

            # Send data to the second destination
            listen_sock.sendto(data, (forward_ip2, forward_port2))
