CFLAGS = -O
CC = g++

main: main.o intkd_tree.o prefix_tree.o tools.o
	$(CC) $(CFLAGS) -o main main.o intkd_tree.o prefix_tree.o tools.o 

main.o: main.cpp
	$(CC) $(CFLAGS) -c main.cpp

intkd_tree.o: intkd_tree.cpp
	$(CC) $(CFLAGS) -c intkd_tree.cpp

prefix_tree.o: prefix_tree.cpp
	$(CC) $(CFLAGS) -c prefix_tree.cpp

tools.o: tools.cpp
	$(CC) $(CFLAGS) -c tools.cpp

clean:
	rm -f main *.o