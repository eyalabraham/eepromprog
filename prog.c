/*
 * prog.c
 *
 *  Created on: Mar 9, 2013
 *      Author: eyal
 *
 *      Purpose:
 *
 *      ATMEL 27C256 32Kx8 eeprom programmer
 *      using IEEE-1284 parallel port interface
 *      using libieee1284 library version: 0.2.11-10build1 (precise)
 *      	/usr/include/ieee1284.h
 *      	/usr/lib/i386-linux-gnu/libieee1284.so
 *
 *      Usage: prog { -r | -w | -x | -q | -h } { -b <bin_file> | -t <S-record_file> } [-s <hex_start_offset>] [-e <hex_end_offset>] [-p <port_id>]
 *
 *      -r	read eeprom
 *      -w	write eeprom
 *		-x  erase device
 *      -q	only query the system: list ieee1284 parallel ports and test programer
 *      -h  print help text
 *      -b	binary file image for read of write
 *      -t	S-record text file for read or write
 *      -s	optional start address/offset, 0x0000 if not provided ** ignored for S-record_file
 *      -e	optional end address/offset, to end of eeprom if not provided ** ignored for S-record_file
 *      -p	use specified ieee1284 port id
 *
 * To Do
 * 1) create S-rec file from EEPROM read
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <ieee1284.h>

/*
 * type definitions
 */
typedef	unsigned char	t_byte;
typedef unsigned short	t_word;

/*
 * function prototypes
 */

// -- programer functions --
int		readEEPROM(void);				// read programer function
int		writeEEPROM(void);				// write programer function
int		eraseEEPROM(void);				// erase eeprom programer function

// -- general functions --
int		writeEEPROMbin(void);			// write eeprom from binary file
int		writeEEPROMsrec(void);			// write eeprom from S-rec file
int		readBlock(t_word, int);			// read a block from eeprom to buffer starting at address
int		writeBlock(t_word, int);		// write block to eeprom from buffer starting at address
int		fileWrite(int, int);			// write data to file, either binary of S-record
int		isProgReady(void);				// return true if programmer passes loop test
int		setAddress(t_word, int);		// set read/write address registers
void	fastByteWrite(t_word, t_byte);	// write byte to address without read verification
int		writeByte(t_word, t_byte);		// write byte to address
t_byte	readByte(t_word);				// read byte from address
void    setStrobe(void);				// set strobe line
void	clrStrobe(void);				// clear strobe line
void	pulseStrobe(void);				// pulse the strobe line
void	selectFunc(int);				// select programer function

/*
 * global definitions
 */
#define VERSION		"v1.0"

#define USAGE		"Usage: prog { -r | -w | -x | -q | -h } [ -b <bin_file> | -t <S-record_file> ]\n" \
					"            [-s <hex_start_offset>] [-e <hex_end_offset>] [-p <port_id>]"
#define HELP		"\n" \
					"\t-r   read EEPROM\n" \
					"\t-w   write EEPROM\n" \
					"\t-x   erase device\n" \
					"\t-q   only query the system: list ieee1284 ports and test programer\n" \
					"\t-h   print help text\n" \
					"\t-b   binary file image for read of write\n" \
					"\t-t   S-record text file for read or write\n" \
					"\t-s   optional start offset, 0x0000 if not provided ** ignored for S-record_file\n" \
					"\t-e   optional end offset, to end of EEPROM if not provided ** ignored for S-record_file\n" \
					"\t-p   optional specified ieee1284 port ID\n"

#define TEXT_LEN	80

#define EEPROM_SIZE	0x8000		// ATMEL 27C256 eeprom size 32Kx8
#define DATA_BUFFER 1024		// 1KB temp data buffer statically allocated

#define DATA_INIT	0xff		// initialize data port
#define CNTRL_INIT	0x0f		// initialize control port

#define	SET_STROBE	0x01		// set strobe bit
#define CLR_STROBE	0xfe		// clear strobe bit

#define CLR_FUNC	0xf1		// clear function bits
#define FUNC_LOADD	0x00		// select low address register
#define FUNC_HIADD	0x02		// select hi address register
#define FUNC_CS		0x02		// select /CS register
#define FUNC_WE		0x04		// select /WE
#define FUNC_OE		0x06		// select /OE
#define FUNC_LOOP	0x0e		// select programmer loop test

#define TEST		0x80		// loopback test mask

#define CS_SET		0x80		// 'or' and 'and' masks for /CS
#define CS_CLR		0x7f

#define DIR_READ	-1			// for use with ieee1284_data_dir()
#define DIR_WRITE	0

#define S_RECORD    1			// data source/destination flags
#define BINARY      2

#define DEF_BIN		"data.bin"	// default binary file
#define DEF_SREC	"data.srec"	// default S-record file

#define READ		1			// programing function
#define WRITE		2
#define ERASE		4
#define QUERY		8

#define WRITEOK		0			// eeprom write byte with no error
#define WRITETOV	1			// eeprom waiting for bit.7 negate time out
#define WRITEVER	2			// eeprom write/verify miscompare

/*
 * globals
 */
struct	parport *port;						// the default ieee1284 programer interface port

t_byte	buffer[DATA_BUFFER];				// data buffer for buffer read and write operations
char	sOutFileName[TEXT_LEN] = DEF_BIN;	// name of binary file
int		nFileFlag = 0;						// S-rec or binary file source/destination

t_word	startAddress = 0;					// programming start addredd
t_word	endAddress = EEPROM_SIZE - 1;		// programming end addredd

/*
 * main function
 */
int main(int argc, char* argv[])
{
	struct	parport_list sysports;				// list of system parallel port

	int		nFlags;								// port open flags
	int		nCapabilities;						// port capability list

	int		nOption = 0;						// command line option parsing
	int		nProgAction = 0;					// programer action
	int		nPortID = 0;						// default port ID for programer

	int		i;
	int		nExitCode = 0;

	printf("%s %s %s\n", VERSION, __DATE__, __TIME__);

	/*
	 * parse command line
	 */
	printf("parsing command line\n");

	if ( argc < 2 )
	{
		printf("\n%s\n", USAGE);
		nExitCode = 1;
		goto ABORT;
	}

	while ( (nOption = getopt(argc, argv, "rwxqhb:t:s:e:p:")) != -1 )
	{
		switch ( nOption )
		{
			case 'r':
				if ( nProgAction == 0 )
					nProgAction = READ;
				else
				{
					printf("too many action switches\n");
					nExitCode = 1;
					goto ABORT;
				}
				break;

			case 'w':
				if ( nProgAction == 0 )
					nProgAction = WRITE;
				else
				{
					printf("too many action switches\n");
					nExitCode = 1;
					goto ABORT;
				}
				break;

			case 'h':
				printf("\n%s\n", USAGE);
				printf("%s", HELP);
				goto ABORT;
				break;

			case 'x':
				if ( nProgAction == 0 )
					nProgAction = ERASE;
				else
				{
					printf("too many action switches\n");
					nExitCode = 1;
					goto ABORT;
				}
				break;

			case 'q':
				if ( nProgAction == 0 )
					nProgAction = QUERY;
				else
				{
					printf("too many action switches\n");
					nExitCode = 1;
					goto ABORT;
				}
				break;

			case 'b':
                if ( nFileFlag == S_RECORD )
                {
                    printf("S-record file '%s' is already defined\n", sOutFileName);
                    nExitCode = 1;
                    goto ABORT;
                }
				strncpy(sOutFileName, optarg, TEXT_LEN-1);
                nFileFlag = BINARY;
				break;

			case 't':
                if ( nFileFlag == BINARY )
                {
                    printf("binary file '%s' is already defined\n", sOutFileName);
                    nExitCode = 1;
                    goto ABORT;
                }
				strncpy(sOutFileName, optarg, TEXT_LEN-1);
                nFileFlag = S_RECORD;
				break;

			case 's':
				sscanf(optarg, "%hx", &startAddress);
				break;

			case 'e':
				sscanf(optarg, "%hx", &endAddress);
				break;

			case 'p':
				sscanf(optarg, "%d", &nPortID);
				break;

			case '?':
				printf("\n%s\n", USAGE);
				nExitCode = 1;
				goto ABORT;
		}
	}

	if ( startAddress > endAddress )		// check for valid start and end addresses
	{
		printf("start address is larger than end address\n");
		nExitCode = 1;
		goto ABORT;
	}

    if ( nFileFlag == 0 )                   // if binary or S-record files were not specified use binary form
    {
    	nFileFlag = BINARY;
    }
    
    /*
     * command line parameter check point. can be commented out later.
     */
	printf("\tfile: '%s'\n", sOutFileName);
    printf("\tfile format 1=srec 2=bin: %d\n", nFileFlag);
	printf("\tstart: 0x%04hx, end: 0x%04hx\n", startAddress, endAddress);
	printf("\tport ID: %d\n", nPortID);

	/*
	 * query the system to find available ports
	 */
	printf("ieee1284_find_ports() ");
	switch ( ieee1284_find_ports(&sysports, 0) )
	{
		case E1284_OK:
			printf("ok\n");
			break;

		case E1284_NOMEM:
		case E1284_NOTIMPL:
			printf("returned an error\n");
			nExitCode = -1;
			break;

		default:
			printf("unspecified error\n");
			nExitCode = -1;
			break;
	}

	if ( nExitCode )
		goto ABORT;

	/*
	 * find ieee1284 ports available in the system
	 */
	printf("found %d ieee1284 port(s)\n",sysports.portc);
	if ( sysports.portc == 0 )
		goto EXIT_NOPORTS;

	if ( nPortID > sysports.portc )
	{
		printf("port ID %d out of range\n", nPortID);
		goto EXIT_NOPORTS;
	}

	for ( i = 0; i < sysports.portc; i++ )
	{
		port = sysports.portv[i];
		printf("\tport ID: %d, name: '%s', at address: 0x%04lx\n", i, port->name, port->base_addr);
	}

	/*
	 * port usage sequence:
	 * 1. open		ieee1284_open()
	 * 2. claim		ieee1284_claim()
	 * 3. do IO work
	 * 4. release	ieee1284_release()
	 * 5. close		ieee1284_close()
	 *
	 */
	nFlags = 0;								// not exclusive use of the port
	nCapabilities = CAP1284_RAW;			// use raw manipulation option

	/*
	 * open port
	 */
	printf("ieee1284_open() ");
	switch ( ieee1284_open(sysports.portv[nPortID], nFlags, &nCapabilities) )
	{
		case E1284_OK:
			printf("ok\n");
			break;

		case E1284_INIT:
			printf("could not initialize or busy\n");
			nExitCode = -1;
			break;

		case E1284_NOTAVAIL:
			printf("capability not available\n");
			nExitCode = -1;
			break;

		case E1284_INVALIDPORT:
			printf("invalid port ID in open\n");
			nExitCode = -1;
			break;

		case E1284_NOMEM:
		case E1284_SYS:
			printf("system error on out of memory\n");
			nExitCode = -1;
			break;

		default:
			printf("unspecified error\n");
			nExitCode = -1;
			break;
	}

	if ( nExitCode )
		goto EXIT_NOOPEN;

	/*
	 * claim port
	 */
	printf("ieee1284_claim() ");
	switch ( ieee1284_claim(sysports.portv[nPortID]) )
	{
		case E1284_OK:
			printf("ok\n");
			break;

		case E1284_NOMEM:
		case E1284_SYS:
			printf("system error on out of memory\n");
			nExitCode = -1;
			break;

		case E1284_INVALIDPORT:
			printf("invalid port ID in open\n");
			nExitCode = -1;
			break;

		default:
			printf("unspecified error\n");
			nExitCode = -1;
			break;
	}

	if ( nExitCode )
		goto EXIT_NOCLAIM;

	/*
	 * port IO
	 *
	 * port bit assignments:
	 *
	 *  data	b7 b6 b5 b4 b3 b2 b1 b0
	 *
	 *  contol	b7 b6 b5 b4 b3 b2 b1 b0
	 *			 |  |  |  |  |  |  |  |
	 *           |  |  |  |  |  |  |  +- Strobe
	 *           |  |  |  |  |  |  +---- F0
	 *           |  |  |  |  |  +------- F1
	 *           |  |  |  |  +---------- F2
	 *           |  |  |  +------------- n.c
	 *           |  |  +---------------- n.c
	 *           |  +------------------- n.c
	 *           +---------------------- n.c
	 *
	 * 		F2 F1 F0
	 * 		0  0  0	... A0 - A7 register clk
	 * 		0  0  1 ... A8 - A14, /CS register clk
	 * 		0  1  0 ... /WE
	 * 		0  1  1 ... /OE
	 * 		1  1  1 ... sys present test (sense on status register b7)
	 *
	 *  status	b7 b6 b5 b4 b3 b2 b1 b0
	 *           |  |  |  |  |  |  |  |
	 *           |  |  |  |  |  |  |  +- n.c
	 *           |  |  |  |  |  |  +---- n.c
	 *           |  |  |  |  |  +------- n.c
	 *           |  |  |  |  +---------- n.c
	 *           |  |  |  +------------- n.c
	 *           |  |  +---------------- n.c
	 *           |  +------------------- n.c
	 *           +---------------------- sys present loopback test
	 *
	 */

	port = sysports.portv[nPortID];			// set global variable to use from this point on

	ieee1284_write_data(port, DATA_INIT);	// initialize programmer
	ieee1284_write_control(port, CNTRL_INIT);
	setAddress(0, CS_SET);

	printf("isProgReady() ");
	if ( isProgReady() )
	{
		printf("ok\n");
		switch ( nProgAction )
		{
			case READ:		// invoke eeprom read to file process
				if ( readEEPROM() )
					printf("eeprom read action failed\n");
				break;

			case WRITE:		// invoke eeprom write process
				if ( writeEEPROM() )
					printf("eeprom write action failed\n");
				break;

			case ERASE:
				eraseEEPROM();
				printf("eeprom erase complete\n");
				break;

			case QUERY:		// nothing else to do here, exit
				printf("programer query ok\n");
				break;

			default:
				printf("command line parsing error\n");
				break;
		}
	}
	else
		printf("failed\n");

	/*
	 * code test -- remove after testing
	 */
	//printf("setAddress() return %d\n", setAddress(0x7fff));


	/*
	 * close and clean-up
	 */
	selectFunc(FUNC_LOOP);
	ieee1284_release(sysports.portv[nPortID]);

EXIT_NOCLAIM:
	ieee1284_close(sysports.portv[nPortID]);

EXIT_NOOPEN:
EXIT_NOPORTS:
	ieee1284_free_ports(&sysports);

ABORT:
	return nExitCode;
}

/*
 * -----------------------------------------
 * ---------  programer functions  ---------
 * -----------------------------------------
 */
 
/*
 * readEEPROM()
 *
 * this function will read eeprom data from
 * 'nStartAddress' to 'nEndAddress' and place read data into
 * a file in either binary image or S-record format
 *
 */
int readEEPROM(void)
{
	int		fd;
	t_word	i;
	int		nCount;
	int		nRead = 0;
	int		nWritten;
	int		nResult = 0;
	int		nTotalRead = 0;

	printf("readEEPOM() started\n");

	if ( setAddress(startAddress, CS_SET) )				// validate address range
	{
		printf("readEEPROM() invalid start address 0x%04hx\n", startAddress);
		return 1;
	}

	if ( (fd = open(sOutFileName, O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR)) > 0 )
	{
		for ( i = startAddress; i <= endAddress; i += (t_word) nRead)
		{
			if ( (endAddress - i + 1) > DATA_BUFFER )
				nCount = DATA_BUFFER;
			else
				nCount = (int) (endAddress - i + 1);

			nRead = readBlock((t_word) i, nCount);	// read a block of data from eeprom to buffer

			if ( nRead != nCount )					// test for address over eeprom size
			{
				printf("readEEPROM() read over EEPROM address range\n");
				nResult = 1;
				break;
			}

			nWritten = fileWrite(fd, nRead);		// write data block from buffer to file

			if ( nRead != nWritten )
			{
				printf("readEEPROM() error writing file\n");
				nResult = 1;
				break;
			}
			else
			{
				nTotalRead += nRead;
				printf("\tread %d bytes\n", nTotalRead);
			}
		}

		close(fd);
	}
	else
	{
		printf("readEEPROM() cound not open file '%s' for writing (errno=%d)\n", sOutFileName, errno);
		nResult = 1;
	}

	return nResult;
}

/*
 * writeEEPROM()
 *
 * this function will write data to eeprom.
 * data will be read from either a binary image file or an S-record file:
 * (1) if data is read from S-record file, then eeprom addresses will be taken from file and 
 * 'nStartAddress' and 'nEndAddress' are ignored.
 * (2) if data is read from a binary image file then data will be written starting at 'nStartAddress'
 * and 'nEndAddress' is ignored.
 *
 */
int writeEEPROM(void)
{
	int	nResult = 0;

	if ( nFileFlag == BINARY )
		nResult = writeEEPROMbin();
	else
		nResult = writeEEPROMsrec();

	return nResult;
}

/*
 * eraseEEPROM()
 *
 * this function will erase the eeprom device.
 *
 */
int eraseEEPROM(void)
{
	t_word	address;

	int		nByteCount = 0;

	for (address = 0; address < EEPROM_SIZE; address++)
	{
		fastByteWrite(address,0xff);					// write blank data pattern

		nByteCount++;

		if ( nByteCount == 64 )							// delay at end of 64 byte block
		{
			nByteCount = 0;
			usleep(20000);
		}

		if ( (address != 0) && (address % 1024) == 0 )	// display progress
			printf("eraseEEPROM() erased %d bytes\n", address);
	}

	setAddress(0, CS_SET);								// negate CS

	return 0;
}

/*
 * -----------------------------------------
 * ----------  general functions  ----------
 * -----------------------------------------
 */

/*
 * writeEEPROMbin()
 *
 * write eeprom with data from binary file
 * start address used to deterine start offset into device
 * end address is ignored
 *
 */
int writeEEPROMbin(void)
{
	/*
     * 1. open file
     * 2. read data from file into buffer 1K or less at the time
     * 3. write data from buffer to eeprom  (start at 'startAddress')
     * 4. repeat until end of file of end of device
     * 5. close file
     *
     */
	int		fd;
	t_word	i;
	int		nCount;
	int		nRead = 0;
	int		nWritten;
	int		nResult = 0;
	int		nFileSize = 0;
	struct	stat filestat;
	int		nTotalWritten = 0;

	printf("writeEEPROMbin() started\n");

	if ( setAddress(startAddress, CS_SET) )				// validate address range
	{
		printf("writeEEPROMbin() invalid start address 0x%04hx\n", startAddress);
		return 1;
	}

	if ( (fd = open(sOutFileName, O_RDONLY)) > 0 )
	{
		if ( fstat(fd, &filestat) == 0 )
			nFileSize = filestat.st_size;
		else
		{
			printf("writeEEPROMbin() error getting file size\n");
			return 1;
		}

		if ( nFileSize > EEPROM_SIZE )
		{
			printf("writeEEPROMbin() file too large to fit in eeprom device\n");
			return 1;
		}

		endAddress = startAddress + (t_word) nFileSize - 1;

		for ( i = startAddress; i <= endAddress; i += (t_word) nWritten)
		{
			if ( (endAddress - i + 1) >= DATA_BUFFER )
				nCount = DATA_BUFFER;
			else
				nCount = (int) (endAddress - i + 1);

			nRead = read(fd, buffer, nCount);			// read a block of data from file

			nWritten = writeBlock((t_word) i, nRead);	// write data to eeprom

			if ( nRead != nWritten )
			{
				printf("writeEEPROMbin() error writing EEPROM at address 0x%x\n", (i + nWritten));
				nResult = 1;
				break;
			}
			else
			{
				nTotalWritten += nWritten;
				printf("\t%d bytes programed\n", nTotalWritten);
			}
		}

		close(fd);
	}
	else
	{
		printf("writeEEPROMbin() could not open file '%s' for reading (errno=%d)\n", sOutFileName, errno);
		nResult = 1;
	}

	return nResult;
}

/*
 * writeEEPROMsrec()
 *
 * write eeprom with data from S-rec file
 * start and end addresses are ignored, device offsets
 * are based on S-rec file
 *
 */
int writeEEPROMsrec(void)
{
	/*
	 * 1. open file
	 * 2. read file one record at a time
	 * 3. extract: byte count, start address, data
	 * 4. set start address and program data bytes
	 * 5. repeat until end of S-record file
	 *
	 * S-record format
	 * SnCCAAAAdddddddddd......ddXX\a
	 *
	 * S0 : Record data sequence contains vendor specific data rather than program data.
	 *      String with file name and possibly version info.
	 * S1, S2, S3: Data sequence, depending on size of address needed.
	 *             16-bit/64K system uses S1, 24-bit address uses S2 and full 32-bit uses S3.
	 * S5: Count of S1, S2 and S3 records previously appearing in the file or transmission.
	 *              The record count is stored in the 2-byte address field.
	 *              There is no data associated with this record type.
	 * S7, S8, S9: The address field of the S7, S8, or S9 records may contain a starting address for the program.
	 *             S7 4-byte address, S8 3-byte address, S9 2 byte address.
	 *
	 */
	FILE	*fp;
	char	*textLine = NULL;
	size_t	len = TEXT_LEN;
	int		nRead;

	char	textTemp[TEXT_LEN] = "0x\0";
	int		nByteCount;
	int		nDataByte;
	int		nTotalWritten = 0;
	int		nWriteResult;

	int		nResult = 0;
	int		i;

	printf("writeEEPROMsrec() started\n");

	if ( (fp = fopen(sOutFileName, "r")) != NULL )				// open S-record file for reading
	{
		while ((nRead = getline(&textLine, &len, fp)) != -1)	// read record from file
		{
			if ( textLine[1] == '1' )							// check and process data record 'S1' only
			{
				strncpy(&textTemp[2], &textLine[2], 2);			// retrieve byte count
				textTemp[4] = '\0';
				sscanf(textTemp, "%x", &nByteCount);
				nByteCount -= 3;								// adjust for address and checksum bytes

				strncpy(&textTemp[2], &textLine[4], 4);			// retrieve write address
				textTemp[6] = '\0';
				sscanf(textTemp, "%hx", &startAddress);

				for ( i = 0; i < nByteCount; i++ )				// loop through data bytes
				{

					strncpy(&textTemp[2], &textLine[8 + (i * 2)], 2);	// get data byte
					textTemp[4] = '\0';
					sscanf(textTemp, "%x", &nDataByte);

					nWriteResult = writeByte((t_word) (i + startAddress), (t_byte) nDataByte);	// write byte to EEPROM

					if ( nWriteResult )							// check for reported errors
					{
						printf("\t==> eeprom write error %d (data=0x%x)\n", nWriteResult, nDataByte);
						break;
					}
				}												// loop on bytes in record

				if ( nWriteResult == 0 )
				{
					nTotalWritten += i;
					printf("writeEEPROMsrec() %d bytes programed\n", nTotalWritten);
				}
			}													// finished processing data record

			if ( nWriteResult )									// break outer loop on error
				break;
		}														// read next record until end of file

		free(textLine);											// free temp record text buffer allocated by getline()
		fclose(fp);												// close file
	}
	else
	{
		printf("writeEEPROMsrec() could not open file '%s' for reading (errno=%d)\n", sOutFileName, errno);
		nResult = 1;
	}

	return nResult;
}

/*
 * readBlock()
 *
 * read a block of data of length 'nCount' from eeprom
 * starting at 'address'.
 * return number of bytes read from eeprom.
 *
 */
int readBlock(t_word address, int nCount)
{
	int	i;
	t_byte	byte;

	for (i = 0; i < nCount; i++)
	{
		if ( (i + address) > (EEPROM_SIZE - 1) )	// test address for out of eeprom size range
			break;

		byte = readByte((t_word) i + address);		// read a byte from eeprom
		buffer[i] = byte;							// store in buffer
	}

	return i;
}

/*
 * writeBlock()
 *
 * write block of 'nCount' bytes from buffer to eeprom
 * starting at 'address'.
 * return number of bytes written to eeprom.
 *
 */
int writeBlock(t_word address, int nCount)
{
	int	i;
	t_byte	byte;
	int nWriteResult;

	for (i = 0; i < nCount; i++)
	{
		if ( (i + address) > (EEPROM_SIZE - 1) )				// test address for out of eeprom size range
			break;

		byte = buffer[i];										// read byte from buffer
		nWriteResult = writeByte((t_word) (i + address), byte);	// write byte to eeprom
		
		if ( nWriteResult )
		{
			printf("\t==> eeprom write error %d (data=0x%x)\n", nWriteResult, byte);
			break;
		}
	}

	return i;
}

/*
 * fileWrite()
 *
 * write data from 'buffer' to file descritor.
 * data will be writted as binary or S-record
 * the function will return the number of bytes writen to the file
 *
 */
int fileWrite(int fd, int nCount)
{
	int	nWritten = 0;

	if ( nFileFlag == BINARY )
	{
		nWritten = write(fd, buffer, nCount);	// write data to binary file
	}
	else
	{
		nWritten = 0;
	}

	return nWritten;
}

/*
 * isProgReady()
 *
 * performe loopback test through parallel port to
 * check presence of programmer.
 * loopback test selects Q7 on 74ls138 and tests state through
 * bit 7 of parallel port's status register.
 *
 */
int isProgReady(void)
{
	t_byte	byte;
	int		nData;
	int		nResult = 0;

	byte = (t_byte) ieee1284_read_control(port);		// make sure loopback test function is set
	byte &= CLR_FUNC;
	byte |= FUNC_LOOP;
	byte |= SET_STROBE;
	ieee1284_write_control(port, (unsigned char) byte);

	nData = ieee1284_read_status(port);		// read status
	if ( nData & TEST )						// loopback bit is '1' then ok
	{
		byte &= CLR_STROBE;
		ieee1284_write_control(port, (unsigned char) byte);	// set loopback bit to '0'
		nData = ieee1284_read_status(port);
		if ( (nData & TEST) == 0 )			// loopback bit is '0' then ok
			nResult = 1;
	}

	ieee1284_write_control(port, CNTRL_INIT);

	return nResult;
}

/*
 * setAddress()
 *
 * set read/write address registers and global address variable
 * return '1' if address is out of range
 *
 */
int setAddress(t_word address, int nCS)
{
	t_byte byte;

	if ( address >= EEPROM_SIZE )
		return 1;								// exit if address is out of range

	byte = (t_byte) (address & 0x00ff);			// extract low address byte
	ieee1284_write_data(port, (unsigned char) byte);
	selectFunc(FUNC_LOADD);
	pulseStrobe();

	byte = (t_byte) ((address & 0xff00) >> 8);	// extract high address byte
	if ( nCS == CS_CLR )						// CS state
		byte &= CS_CLR;
	else
		byte |= CS_SET;
	ieee1284_write_data(port, (unsigned char) byte);
	selectFunc(FUNC_HIADD);
	pulseStrobe();

	return 0;
}

/*
 * fastByteWrite()
 *
 * write 'byte' to 'address' without write verification
 * use to load special eeprom commands
 *
 */
void fastByteWrite(t_word address, t_byte byte)
{
	setAddress(address, CS_CLR);						// setup write address and assert CS

	selectFunc(FUNC_WE);								// select eeprom /WE function
	ieee1284_write_data(port, (unsigned char) byte);	// write data
	pulseStrobe();										// pulse /WE line to program
}

/*
 * void	writeByte(int);
 *
 * write a byte to the eeprom at 'address'
 * return error code on write time-out of verify error, otherwise
 * returns '0'
 *
 */
int writeByte(t_word address, t_byte byte)
{
	int	i = 0;
	t_byte readTest = 1;
	t_byte readBack;
	int nResult = WRITEOK;
    
	setAddress(address, CS_CLR);						// setup write address and assert CS

	selectFunc(FUNC_WE);								// select eeprom /WE function
	ieee1284_write_data(port, (unsigned char) byte);	// write data
	pulseStrobe();										// pulse /WE line to program

	usleep(1000);

	setAddress(address, CS_SET);						// negate CS

	while ( readTest )									// read-test for inverted I/O7 bit
	{
		readBack = readByte(address);					// read back the byte
		
		readTest = readBack;							// isolate bit.7
		readTest ^= byte;
		readTest &= 0x80;

		//printf("readBack %x, readTest %x\n", readBack, readTest);

		i++;											// crude way to escape on a timeout
		if ( i > 100 )									// typical write time is about 40 cycles
		{
			nResult = WRITETOV;							// timed out while waiting for bit.7 to negate
			break;
		}
	}

	if ( (nResult == WRITEOK) && (readBack != byte) )	// check if write is good and verified
		nResult = WRITEVER;

	return nResult;
}

/*
 * readByte()
 *
 * read a byte from the eeprom at 'address'
 *
 */
t_byte readByte(t_word address)
{
	t_byte byte;

	setAddress(address, CS_CLR);				// setup read address and assert CS

	ieee1284_data_dir(port, DIR_READ);			// disable port line drivers

	selectFunc(FUNC_OE);						// select eeprom /OE function
	clrStrobe();								// activate /OE
	usleep(10);
	byte = (t_byte) ieee1284_read_data(port);	// read data
	setStrobe();								// deactivate /OE

	ieee1284_data_dir(port, DIR_WRITE);			// enable port line drivers

	setAddress(address, CS_SET);				// negate CS

	return byte;
}

/*
 * setStrobe()
 *
 * set strobe line high
 *
 */
void setStrobe(void)
{
	unsigned char byte;

	byte = ieee1284_read_control(port);

	byte |= SET_STROBE;
	ieee1284_write_control(port, byte);
}

/*
 * clrStrobe()
 *
 * set strobe line low
 *
 */
void clrStrobe(void)
{
	unsigned char byte;

	byte = ieee1284_read_control(port);

	byte &= CLR_STROBE;
	ieee1284_write_control(port, byte);
}

/*
 * pulseStrobe()
 *
 * pulse the strobe line
 *
 */
void pulseStrobe(void)
{
	setStrobe();
	clrStrobe();
	setStrobe();
}

/*
 * selectFunc()
 *
 * select programer function
 *
 */
void selectFunc(int nFunc)
{
	t_byte byte;

	byte = (t_byte) ieee1284_read_control(port);
	byte &= CLR_FUNC;
	byte |= (t_byte) nFunc;
	ieee1284_write_control(port, (unsigned char) byte);
}
