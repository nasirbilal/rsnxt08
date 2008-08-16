//testing writing to text files
TFileHandle hFileHandle = 0;
TFileIOResult nIoResult;
short nFileSize = 100;
const string sFileName = "gyro.dat";


task main()
{
	Delete(sFileName, nIoResult);
	OpenWrite(  hFileHandle, nIoResult, sFileName, nFileSize);
	if (nIoResult != ioRsltSuccess)
	PlaySound(soundLowBuzz);
	else
	{
		int x[10] = {0,1,2,3,4,5,6,7,8,9};
		int y[10] = {0,1,2,3,4,5,6,7,8,9};
		char nIndex;
		for (nIndex = 0; nIndex < 10; ++nIndex)
		{
			WriteShort(  hFileHandle, nIoResult, x[nIndex]);
			WriteByte(  hFileHandle, nIoResult, '\n');
			WriteShort(hFileHandle, nIoResult, y[nIndex]+1);
			WriteByte(hFileHandle, nIoResult, '\n');
			if (nIoResult != ioRsltSuccess)
			{
				PlaySound(soundLowBuzz);
				break;
			}
		}
	}
	Close(hFileHandle, nIoResult);
  OpenRead(hFileHandle, nIoResult, sFileName, nFileSize);
  if (nIoResult != ioRsltSuccess)
	PlaySound(soundLowBuzz);
	else
	{
		short x;
		short y;
		byte separator;
		char nIndex;
		while(nIoResult != ioRsltEndOfFile)
		{
			ReadShort(  hFileHandle, nIoResult, x);
			ReadByte(hFileHandle, nIoResult, separator);
			ReadShort(  hFileHandle, nIoResult, y);
			ReadByte(hFileHandle, nIoResult, separator);
			if (nIoResult != ioRsltSuccess)
			{
				PlaySound(soundLowBuzz);
				break;
			}
			nxtDisplayCenteredBigTextLine(1, "x:%d", x);
			nxtDisplayCenteredBigTextLine(4, "y:%d", y);
		  wait10Msec(100);
		}

	}
 	Close(hFileHandle, nIoResult);

}
