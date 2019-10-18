CXX = g++
PROGRAM = bin/main
CXXFILES = $(wildcard *.cpp)
OBJS = $(CXXFILES:.cpp=.o)
DEPDIR = .deps
CXXFLAGS = `pkg-config --cflags gtkmm-2.4` -std=c++17 -Wall -g
LIBS = `pkg-config --libs gtkmm-2.4`

all: $(PROGRAM)

-include $(OBJS:%.o=$(DEPDIR)/%.Po)

%.o: %.cpp
	@mkdir -p $(DEPDIR)
	$(CXX) -MT $@ -MD -MP -MF $(DEPDIR)/$*.Tpo -c -o $@ $(CXXFLAGS) $<
	@mv -f $(DEPDIR)/$*.Tpo $(DEPDIR)/$*.Po

$(PROGRAM): $(OBJS)
	$(CXX) -o $(PROGRAM) $(OBJS) $(LIBS)

clean:
	rm -f $(OBJS)
	rm -f $(PROGRAM)