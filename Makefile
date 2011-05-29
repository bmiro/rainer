all: libs
	g++ -c rainer.cpp -I/usr/local/Aria/include
	g++ rainer.o librainer.o -o rainer -L/usr/local/Aria/lib/ -fabi-version=1 -lrt -Llib -lAria -lpthread -ldl -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic

libs: lib

lib:
	g++ -c librainer.cpp -I/usr/local/Aria/include

wander:
	g++ -c wander.cpp -I/usr/local/Aria/include
	g++ wander.o -o wander -L/usr/local/Aria/lib/ -lrt -Llib -lAria -lpthread -ldl -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic
2:
	g++ -c rainer2.cpp -I/usr/local/Aria/include
	g++ rainer2.o -o rainer2 -L/usr/local/Aria/lib/ -lrt -Llib -lAria -lpthread -ldl -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic
3:
	g++ -c rainer3.cpp -I/usr/local/Aria/include
	g++ rainer3.o -o rainer3 -L/usr/local/Aria/lib/ -lrt -Llib -lAria -lpthread -ldl -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic

sim:
	#xterm -hold -e "MobileSim -m /usr/local/Aria/maps/triangle.map" &
	xterm -hold -e "MobileSim -m ./maps/quatrepunobs.map" &
	#xterm -hold -e "MobileSim -nomap" &
	sleep 1
	xterm -hold -e "./rainer"

kill:
	killall -9 MobileSim

clean:
	rm *.o
