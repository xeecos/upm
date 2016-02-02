%module javaupm_makeblock
 
%include "../upm.i"
 
%include "typemaps.i"
 
 
%{
 
    #include "makeblock.h"
%}
 
 
%include "makeblock.h"
 
 
%pragma(java) jniclasscode=%{
 
    static {
        try {
            System.loadLibrary("javaupm_makeblock");
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Native code library failed to load. \n" 
+ e);
            System.exit(1);
        }
    }
%}
