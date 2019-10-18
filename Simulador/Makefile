CXX = g++
PROGRAM = bin/main
CXXFILES = $(wildcard *.cpp)
OBJS = $(CXXFILES:.cpp=.o)
DEPDIR = .deps
CXXFLAGS = `pkg-config --cflags gtkmm-2.4` -std=c++17 -Wall -g
LIBS = `pkg-config --libs gtkmm-2.4` -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_flann

all: $(PROGRAM)

-include $(OBJS:%.o=$(DEPDIR)/%.Po) 

%.o: %.cpp
	@mkdir -p $(DEPDIR)
	$(CXX) -I/usr/include/opencv -MT $@ -MD -MP -MF $(DEPDIR)/$*.Tpo -c -o $@ $(CXXFLAGS) $<
	@mv -f $(DEPDIR)/$*.Tpo $(DEPDIR)/$*.Po

$(PROGRAM): $(OBJS)
	$(CXX) -o $(PROGRAM) $(OBJS) $(LIBS) -L/usr/lib/x86_64-linux-gnu


clean:
	rm -f $(OBJS)
	rm -f $(PROGRAM)