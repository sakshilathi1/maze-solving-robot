% Main script
server_ip = '192.168.1.159';
server_port = 5001;
message_home = 'set_angles(10.0,11.0,12.2,12.3,11.1,16.0,500)';

% Call the function
send_tcp_packet(server_ip, server_port, message_home);


