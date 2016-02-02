%module javaupm_mymodule
 
%include "../upm.i"
 
%include "typemaps.i"
 
 
%{
 
    #include "mymodule.h"
%}
 
 
%include "mymodule.h"
 
 
%pragma(java) jniclasscode=%{
 
    static {
        try {
            System.loadLibrary("javaupm_mymodule");
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Native code library failed to load. \n" 
+ e);
            System.exit(1);
        }
    }
%}
