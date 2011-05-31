all: libs
	g++ -Wall -c rainer.cpp -I/usr/local/Aria/include
	g++ -Wall libtact.o librainermap.o libtrace.o librainer.o lib2d.o rainer.o -o rainer -L/usr/local/Aria/lib/ -fabi-version=1 -lrt -Llib -lAria -lpthread -ldl -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic

libs: librainer librainermap libtrace lib2d libtact

librainer:
	g++ -Wall -c librainer.cpp -I/usr/local/Aria/include

librainermap:
	g++ -Wall -c librainermap.cpp

libtrace:
	g++ -Wall -c libtrace.cpp

libtact:
	g++ -c libtact.cpp -I/usr/local/Aria/include  -Wall

lib2d:
	g++ -c lib2d.cpp -Wall

provalib2d: lib2d
	g++ -c provesLlib2d.cpp -Wall
	g++ lib2d.o provesLlib2d.o -o l2d -Wall

sim:
	#xterm -hold -e "MobileSim -m /usr/local/Aria/maps/triangle.map" &
	xterm -hold -e "MobileSim -m ./maps/quatrepunobs.map" &
	#xterm -hold -e "MobileSim -nomap" &
	#xterm -hold -e "MobileSim -m ./maps/obstacleApunt.map" &
	sleep 1
	xterm -hold -e "./rainer"

kill:
	killall -9 MobileSim

clean:
	rm *.o
