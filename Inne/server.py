import socket
from time import sleep
import pickle
class Server:
	def __init__(self, ip, port):	#konstruktor tworzy socket oraz łączy i testuje połączenie z serverem
		self.server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		self.port = port
		address = (ip, self.port)
		self.server.bind(address)
		self.server.listen(2)
		self.statusFlag = False
		self.client, addr = self.server.accept()
		msg ='Connection succesful'
		self.client.send(pickle.dumps(msg))
					
				
	
	def receiveData(self): 	#metoda, którą odpalimy w wątku i będzie odbierać napływające dane z jetsona
		data = self.client.recv(4096)
		return pickle.loads(data)
	
	def sendData(self, data):
		self.client.send(pickle.dumps(data))
