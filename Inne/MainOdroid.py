from connectionOdroid import *

IP_ADDRESS_1 = '10.42.0.158'  # address jetson
IP_ADDRESS_2 = '192.168.137.208'  # address odroid

PORT = 8181


conn = Connection(IP_ADDRESS_2, PORT)
conn.start()
