A quick note about building Kinematic.
  o This version uses Cygwin build tools.
    (www.cygwin.org - I downloaded the complete set which is overkill)
  o Set the environment variable MAKEFILES to point to Hierarchy.mak
    (My Computer-->properties-->advanced-->environment variables)

The make files are a bit unconventional. I got really frustrated
with Visual Studio's management of projects, and I wasted a lot of
time trying to get Jam to work. So I created GNU make scripts which
"sort of" do what I want.
  o They start by searching upwards and including 
    the first "Project.mak" file they find. 
  o Then they restart their upwards search and include 
    the first "Makefile.mak".
  o There are macros for finding all source files in a src tree,
    and another for compiling them into a library.

The idea is to maintain a directory tree of source files without 
explicitly listing the file names or creating intermediate
make files. 

Currently, the make files are applied only to the library,
but I'd like to extend it to the test programs as well as the
applications. My goal is add a new test program or library directory
and not have to update the makefiles.

So, to compile,
    bash>  export MAKEFILES=<path>Hierarchy.mak
    bash>  cd Library
    bash>  make
    bash>  cd Applications
    bash>  make

The test directory isn't in the scheme of things yet, but 
it will be. Now that I can create test programs without creating
new VS projects, I expect there will be a lot of them.

I like the VC++ debugger, so I may switch back and forth
with their compiler. Or maybe use Eclipse.

