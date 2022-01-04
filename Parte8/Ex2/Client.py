#!/usr/bin/env python3
# --------------------------------------------------
# Miguel Riem Oliveira.
# PSR, September 2020.
# Adapted from https://stackabuse.com/basic-socket-programming-in-python/
# -------------------------------------------------
import socket
import time
import dog_lib

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # create TCP/IP socket
local_hostname = socket.gethostname()  # retrieve local hostname
local_fqdn = socket.getfqdn()  # get fully qualified hostname
ip_address = socket.gethostbyname(local_hostname)  # get the according IP address

server_address = (ip_address, 23452)  # bind the socket to the port 23456, and connect
sock.connect(server_address)
print ("connecting to %s (%s) with %s" % (local_hostname, local_fqdn, ip_address))

#Create a dog instance to transmit
dog = dog_lib.Dog(name='Toby', age=7, color='brown')  # instantiate a new dog
dog.addBrother('Lassie')
dog.addBrother('Boby')
print(dog)

#Create message by serialising dog to a list
messages = []
messages.append(dog.name)
messages.append(dog.color)
messages.append(str(dog.age))
for brother in dog.brothers:
    messages.append(brother)

message_to_send = ','.join(messages)
print(message_to_send)

message_formated = str(message_to_send).encode("utf-8")
sock.sendall(message_formated)
time.sleep(2)  # wait for two seconds

sock.close()  # close connection
