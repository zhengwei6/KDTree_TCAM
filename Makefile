CFLAGS = -O
CC = g++

main: main.o tcam_m3_kd_tree.o classic_kd_tree.o intkd_tree.o prefix_tree.o tools.o
	$(CC) $(CFLAGS) -o main main.o tcam_m3_kd_tree.o classic_kd_tree.o intkd_tree.o prefix_tree.o tools.o

main.o: main.cpp
	$(CC) $(CFLAGS) -c main.cpp

intkd_tree.o: intkd_tree.cpp
	$(CC) $(CFLAGS) -c intkd_tree.cpp

classic_kd_tree.o: classic_kd_tree.cpp intkd_tree.h
	$(CC) $(CFLAGS) -c classic_kd_tree.cpp

tcam_m3_kd_tree.o: tcam_m3_kd_tree.cpp intkd_tree.h
	$(CC) $(CFLAGS) -c tcam_m3_kd_tree.cpp
	
prefix_tree.o: prefix_tree.cpp
	$(CC) $(CFLAGS) -c prefix_tree.cpp

tools.o: tools.cpp
	$(CC) $(CFLAGS) -c tools.cpp


clean:
	rm -f main *.o