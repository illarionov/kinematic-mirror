APPS:= NtripLogger NtripAc12  Process Acquire

all: $(addprefix $(BINDIR), $(APPS))

$(BINDIR)% : %.cpp  $(BINDIR)Kinematic.a
	$(CXX) $^ $(CPPFLAGS) $(LDFLAGS) $(CPPOOPT) $(LDOPT) -o $@

$(BINDIR)kinematic.a :
	(cd $(PROJECT_ROOT)/Library; make)


