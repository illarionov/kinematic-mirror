all:  Library Applications Tests

Library: 
	(cd Library; make -j10)

Applications: 
	(cd Applications; make)

Tests: 
	(cd Tests; make)

clean:
	rm -rf $(OBJDIR) $(BINDIR)

.PHONY: all Library Applications Tests clean
