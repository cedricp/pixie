// ********************************************************
// ** DWARF-LABS PTC Filter
// ** (c) 2010 Cédric PAILLE
// ********************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct color	 { float r, g, b; };

extern int
SSDiffusion(char *inputfilenames[], char *outputfilename, int numfiles,
	    int albedoFromFile, struct color *albedo,
	    int dmfpFromFile, struct color *diffusemeanfreepath,
	    float unitlength, float ior,
	    float maxsolidangle, int printProgress);

extern int
GIDiffusion(char *inputfilenames[], char *outputfilename, int numfiles,
	    float maxsolidangle, int printProgress);

void printUsage(){
    printf("Options :\n  --filter (sss)\n  --albedo (floatR float G float B\n  --dmfp  (floatR floarG floatB)\n");
}

void printVersion(){

}

int main(int argc, char *argv[])
{

    char* file;
    char *files[2];
    files[0] = file;
    char *outFile;
    int progress = 0;
    const char *filterType = "undefined";


	struct color albedo;
	struct color dmfp;
	albedo.r = 0.830;
	albedo.g = 0.791;
	albedo.b = 0.753;

	dmfp.r = 8.51;
	dmfp.g = 5.57;
	dmfp.b = 3.95;

	float maxsolidangle = .5;
	float unitlength    = 1.0;
	float ior           = 1.5;

	for (int i=1;i<argc;i++) {
		if (strcmp(argv[i],"-h") == 0
			|| strcmp(argv[i],"-help") == 0
			|| strcmp(argv[i],"--help") == 0) {
			printUsage();
			exit(0);
		} else if (strcmp(argv[i],"-v") == 0
				   || strcmp(argv[i],"-version") == 0
				   || strcmp(argv[i],"--version") == 0) {
			printVersion();
			exit(0);
        } else if (strcmp(argv[i],"--filter") == 0){
            i++;

            if( i < argc ){
                filterType = argv[i];
            }
        } else if (strcmp(argv[i],"--albedo") == 0){
            i++;

            if( i < argc + 2 ){
                albedo.r = atof(argv[i]);
                albedo.g = atof(argv[i+1]);
                albedo.b = atof(argv[i+2]);
                i+=2;
            }
        } else if (strcmp(argv[i],"--dmfp") == 0){
            i++;

            if( i < argc + 2 ){
                dmfp.r = atof(argv[i]);
                dmfp.g = atof(argv[i+1]);
                dmfp.b = atof(argv[i+2]);
                i+=2;
            }
        } else if (strcmp(argv[i],"--maxsolidangle") == 0){
            i++;

            if( i < argc ){
                maxsolidangle = atof(argv[i]);
            }
        } else if (strcmp(argv[i],"--unitlength") == 0){
            i++;

            if( i < argc ){
                unitlength = atof(argv[i]);
            }
        } else if (strcmp(argv[i],"--ior") == 0){
            i++;

            if( i < argc ){
                ior = atof(argv[i]);
            }
        } else if (strcmp(argv[i],"--progress") == 0){
            i++;

            if( i < argc ){
                progress = atoi(argv[i]);
            }
        } else {
            file = argv[i];
            i++;
            if (i < argc){
                outFile = argv[i];
            }
        }
	}

    if ( strcmp(filterType,"sss") == 0){
        SSDiffusion(&file,outFile,1,0,&albedo,0,&dmfp,unitlength,ior,maxsolidangle,progress);
    } else if ( strcmp(filterType,"colourbleeding") == 0){
        GIDiffusion(&file,outFile,1,maxsolidangle,progress);
    }else{
        printf("No filter type, doing nothing\n");
    }

    return 0;
}
