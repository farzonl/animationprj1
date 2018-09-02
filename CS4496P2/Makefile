CXX    = clang++
CFLAGS = -O2 -std=c++11
INCLUDES = -I ./

TARGET = main.out
ifeq ($(OS),Windows_NT) 
	LIBS_GL = -lfreeglut -lglu32 -lopengl32		#Windows
	TARGET = main.exe	
else ifeq ($(shell uname -s),Darwin)	
	LIBS_GL = -framework OpenGL -framework GLUT	#Mac
else	
	LIBS_GL = -lglut -lGL -lGLU			#Linux(defalut)
endif
OBJS = main.o \
	particle.o \
	simulator.o \
    equations.o  \
	Timer.o

all: $(TARGET)
					
$(TARGET): $(OBJS)
	$(CXX) $(CFLAGS)  -o $@ $(OBJS) $(LIBS_GL)

clean:
	-rm -f $(OBJS)
.cpp.o:
	$(CXX) $(CFLAGS) $(INCLUDES) -c $<
