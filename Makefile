all:
	g++ -c rainer.cpp -I/usr/local/Aria/include
	g++ rainer.o -o rainer -L/usr/local/Aria/lib/ -lrt -Llib -lAria -lpthread -ldl -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic

1:
	g++ -c rainer1.cpp -I/usr/local/Aria/include
	g++ rainer1.o -o rainer1 -L/usr/local/Aria/lib/ -lrt -Llib -lAria -lpthread -ldl -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic
2:
	g++ -c rainer2.cpp -I/usr/local/Aria/include
	g++ rainer2.o -o rainer2 -L/usr/local/Aria/lib/ -lrt -Llib -lAria -lpthread -ldl -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic
3:
	g++ -c rainer3.cpp -I/usr/local/Aria/include
	g++ rainer3.o -o rainer3 -L/usr/local/Aria/lib/ -lrt -Llib -lAria -lpthread -ldl -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic

sim:
	MobileSim -m /usr/local/Aria/maps/triangle.map &

kill:
	killall -9 MobileSim

