//testing writing to text files
TFileHandle hFileHandle = 0;
TFileIOResult nIoResult;
short nFileSize = 1000;
const string sFileName = "local.dat";


task main()
{
	Delete(sFileName, nIoResult);
	OpenWrite(  hFileHandle, nIoResult, sFileName, nFileSize);
	if (nIoResult != ioRsltSuccess)
	PlaySound(soundLowBuzz);
	else
	{

			WriteShort(  hFileHandle, nIoResult, 1);

			if (nIoResult != ioRsltSuccess)
			{
				PlaySound(soundLowBuzz);

			}

	}


	//if (nIoResult != ioRsltSuccess)
	//PlaySound(soundLowBuzz);
//	else
//	{

			WriteShort(  hFileHandle, nIoResult, 2);
			if (nIoResult != ioRsltSuccess)
			{
				PlaySound(soundLowBuzz);

			}

//	}
		//Close(hFileHandle, nIoResult);
	OpenRead(hFileHandle, nIoResult, sFileName, nFileSize);

	short test;
	short test2;
		short test3;
			short test4;
			short test5;


		if (nIoResult != ioRsltSuccess)
	PlaySound(soundLowBuzz);
	else
	{
	 Readshort(hFileHandle, nIoResult, test);
	 Readshort(hFileHandle, nIoResult, test2);

	}
	nxtDisplayCenteredTextLine(1,"%d",test);
	nxtDisplayCenteredTextLine(2,"%d",test2);
	wait10Msec(100);
  //hFileHandle = 0;
	//Close(hFileHandle, nIoResult);
 // OpenWrite(hFileHandle, nIoResult, sFileName, nFileSize);
  WriteShort(  hFileHandle, nIoResult, 4);

  Readshort(hFileHandle, nIoResult, test);
	 Readshort(hFileHandle, nIoResult, test2);
Readshort(hFileHandle, nIoResult, test3);
nxtDisplayCenteredTextLine(1,"%d",test);
	nxtDisplayCenteredTextLine(2,"%d",test2);
	nxtDisplayCenteredTextLine(3,"%d",test3);
	wait10Msec(100);
	WriteShort(  hFileHandle, nIoResult, 4);
  Readshort(hFileHandle, nIoResult, test5);
  nxtDisplayCenteredTextLine(1,"%d",test5);
	wait10Msec(100);
Close(hFileHandle, nIoResult);



}
//////////////NOTE THIS ALLOWS ME TO READ AND WRITE TO AN OPEN FILE _ YEAH _ JUST NEED TO PUT INTO LOCAL AND TEST__________
