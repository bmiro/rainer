all: libs
	g++ -c rainer.cpp -I/usr/local/Aria/include -Wall 
	g++ libtact.o librainermap.o libtrace.o librainer.o lib2d.o rainer.o -o rainer -L/usr/local/Aria/lib/ -fabi-version=1 -lrt -Llib -lAria -lpthread -ldl -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic -Wall 
	

libs: libtact librainer librainermap libtrace lib2d 

libtact:
	g++ -c libtact.cpp -I/usr/local/Aria/include  -Wall

librainermap:
	g++ -c librainermap.cpp -Wall

libtrace:
	g++ -c libtrace.cpp -Wall 

librainer:
	g++ -c librainer.cpp -I/usr/local/Aria/include -Wall 

lib2d:
	g++ -c lib2d.cpp -Wall

provalib2d: lib2d
	g++ -c provesLlib2d.cpp -Wall
	g++ lib2d.o provesLlib2d.o -o l2d -Wall

sim:
	#xterm -hold -e "MobileSim -m /usr/local/Aria/maps/triangle.map" &
	#xterm -hold -e "MobileSim -m ./maps/quatrepunobs.map" &
	xterm -hold -e "MobileSim -m ./maps/obstacleInclinat.map" &
	#xterm -hold -e "MobileSim -m ./maps/duesparets.map" &
	#xterm -hold -e "MobileSim -nomap" &
	#xterm -hold -e "MobileSim -m ./maps/provaSensors.map" &
	sleep 1
	xterm -hold -e "./rainer"

kill:
	killall -9 MobileSim

clean:
	rm *.o
