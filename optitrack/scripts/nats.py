import natnetclient as natnet
client = natnet.NatClient(client_ip='192.168.0.5', data_port=1511, comm_port=1510)

client.start_recording()