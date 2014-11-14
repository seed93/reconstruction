#include "CReconstruction.h"
// main.cpp
// put as few as codes here

int main(int Argc, char ** Argv)
{
	clock_t start = clock();
	CReconstrction recon;
	char config_file[100];
	if (Argc == 1)
		strcpy(config_file, "config.yml");
	else strcpy(config_file, Argv[1]);
	if (recon.Init(config_file) == false)
		return -1;
//	recon.m_PreProcess.Process(2);

 	recon.m_Matching.MatchAllLayer();
 	printf("Matching time: %.3f s\n",double(clock()-start)/1000.);
//	recon.m_CloudOptimization.filter();
#ifdef IS_PCL
	recon.m_CloudOptimization.run();
#endif

	printf("total time: %.3f s\n",double(clock()-start)/1000.);
	return 1;
}
