OPTS = -g -Wall -O3 -std=c++0x

maze: maze_bb.cc
	g++ ${OPTS} -o maze_bb maze_bb.cc	 

all: maze_bb 

clean: 
	rm maze_bb
 
