all:
	g++ -c rainer.cpp -I/usr/local/Aria/include
	g++ rainer.o -o rainer -L/usr/local/Aria/lib/ -lrt -Llib -lAria -lpthread -ldl -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic

sim:
	#/usr/local/Aria/bin/SRIsim & ./rainer
	MobileSim -m /usr/local/Aria/maps/triangle.map & sleep 3 && ./rainer
	
