CC	= g++
CCFLAGS	= -c
LDFLAGS	= -g++
LIBS	=

MYSOURCE	= ${wildcard *.cpp}
MYOBJECTS	= ${MYSOURCE:.cpp=.o}
BIN		= servoControl

all:	$(MYOBJECTS) $(BIN)

$(BIN):	$(MYOBJECTS) Makefile
	$(CC)  $(MYOBJECTS) $(LDFLAGS) -o $(BIN)

$(MYOBJECTS):	$(MYSOURCE)
	$(CC) $(CCFLAGS) $(MYSOURCE)

