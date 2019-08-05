CFLAGS = -coverage -O2 -std=c++11
INCLUDES = -I ./

TARGET = main.out
ifeq ($(OS),Windows_NT) 
	LIBS_GL = -lfreeglut -lglu32 -lopengl32		#Windows
	TARGET = main.exe	
else ifeq ($(shell uname -s),Darwin)	
	CXX := clang++
	LIBS_GL = -framework OpenGL -framework GLUT	#Mac
else	
	CXX := g++
	LIBS_GL = -lglut -lGL -lGLU			#Linux(defalut)
endif
OBJS = main.o \
	particle.o \
	simulator.o \
    equations.o  \
	Timer.o

all: initDep
all: $(TARGET)

initDep :
	bash uzDep.sh

$(TARGET): $(OBJS)
	$(CXX) $(CFLAGS)  -o $@ $(OBJS) $(LIBS_GL)

clean:
	-rm -f $(OBJS) *.gcov *.gcda *.gcno
.cpp.o:
	$(CXX) $(CFLAGS) $(INCLUDES) -c $<
