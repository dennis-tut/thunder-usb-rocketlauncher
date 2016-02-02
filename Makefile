# Autor:	Maximilian Kaul
# E-Mail:	post@maxkaul.de

# M A C R O S

# Version
VERSION = 1.1

AUTHOR = "Maximilian Kaul post@maxkaul.de"

# Source files
SOURCES = $(wildcard *.cpp)

# Name of the application
EXE_NAME = USBMissile

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# C++ Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -std=c++11 -pedantic -pthread -O2 -DVERSION=\"$(VERSION)\" -DAUTHOR=\"$(AUTHOR)\"

# Link
LDFLAGS = `pkg-config opencv libusb-1.0 --libs`

# Include
INCLUDE = `pkg-config opencv libusb-1.0 --cflags`

# T A R G E T S

all: $(SOURCES) $(EXE_NAME)

$(EXE_NAME): $(OBJECTS) 
	$(CXX) $(CXXFLAGS) -o $@ $(OBJECTS) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $<

# Remove executable, object files and emacs autosave files
.PHONY: clean
clean:
	rm -f ${EXE_NAME} ${OBJECTS} *~ *.gch *.o *.aux *.log *.nav *.out *.snm *.toc

# tar
tar:
	# Lösche alten tar-Ball
	rm -rf ${EXE_NAME}.tar.gz
	# alles packen
	tar czvf ${EXE_NAME}.tar.gz *.cpp *.hpp Makefile README *.pdf
