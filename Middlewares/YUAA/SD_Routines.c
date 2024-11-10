/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file SD_Routines.c
* @brief C File with routines for SD card
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Rome T., Elijah B.
* @version           1.0.1
* @date              2022.05.23
*
* @details           Routines to be used with tasks interfacing with the SD card
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "SD_Routines.h"
#include "system_manager.h"
#include <string.h>
#ifdef DEBUG_ENABLED
#include "AppTasks.h"
#include "MCU_Init.h"
#include "FreeRTOS.h"
#endif 


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#define errorcheck_goto(fres, lbl) if ((FRESULT)fres != FR_OK) goto lbl
#define errorcheck_ret(fres) if ((FRESULT)fres != FR_OK) return fres

// Macro to keep the first error encountered and discard any later errors
#define keep_first_error(old_fres, new_fres) if (old_fres == FR_OK) old_fres = new_fres

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL (NON STATIC) ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifdef DEBUG_ENABLED
/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief List contents of a directory
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      path - string containing path to the directory to be listed
* @param[output]     output - array of FILINFO objects to be populated; numFiles - maximum
*                    number of files to be listed, if the actual number of files found in 
*                    the directory is smaller, the next FILINFO object in output is set 
*                    with fname = "end\0"
* @return            FRESULT of operation
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT SD_listDir(const char* path, FILINFO * output, uint16_t numFiles)
{
    DIR dir; FILINFO fno; FRESULT fres;
    fres = f_opendir(&dir, path); errorcheck_ret(fres); //open directory
    
    uint16_t i = 0;
    for (; i < numFiles; i++)
    {
        f_readdir(&dir, &fno); //read next file into fno
        if (fno.fname[0] != '\0') output[i] = fno;
        else break;
    }

    if (i < numFiles) //listed all files in directory
         strcpy(output[i].fname, "end"); // output[i + 1].fname = "end\0";

    fres = f_closedir(&dir);
    return fres;
}
#endif

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Create a file
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      filename - name of file to be created (including filetype extension)
* @param[output]     none
* @return            FRESULT of operation
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT SD_createFile(const char* filename)
{
    FIL fno; FRESULT fres;

    fres = f_open(&fno, filename, FA_CREATE_NEW); errorcheck_ret(fres); //create the file

    fres = f_close(&fno); //close the file
    return fres;
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get free space remaining on SD card
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      none
* @param[output]     space - pointer to integer that will contain free space remaining on SD 
                     card in KiB
* @return            FRESULT of operation
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT SD_getFreeSpace(uint32_t* space)
{
    FRESULT fres;
    DWORD freeClusters, freeSectors;
    FATFS* fs;

    //get number of free clusters to freeClusters and 
    //filesystem info to fs
    fres = f_getfree("", &freeClusters, &fs);  
    errorcheck_ret(fres);
    //calculate number of free sectors based on cluster 
    //size as stored in filesystem info object
    freeSectors = freeClusters * fs->csize; 

    //calculate amount of free KiB based on size of each sector
    //as defined in ffconf.h
    *space = freeSectors * (_MIN_SS / 1024.0); 

    return fres;
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get contents of a file at a specific offest
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      filename - name of file to be read (including filetype extension);
*                    offset - byte offset from the top of the file to advance the file pointer
* @param[output]     output - pointer to buffer which sould be filled with file contents;
*                    numChars - maximum number of characters to be written to output
* @return            FRESULT of operation. If the file is not long enough for the requested
*                    read operation, no data is read and `FR_INVALID_PARAMETER' is returned.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT SD_getFileContentsOffset(const char* filename, FSIZE_t offset, void* output, UINT numChars)
{
    FIL fno; FRESULT fres, fres_temp;
    fres = f_open(&fno, filename, FA_READ | FA_OPEN_EXISTING); //open the file for reading
    errorcheck_ret(fres);

    if (offset + numChars > f_size(&fno)) { fres = FR_INVALID_PARAMETER; goto end_closefile; }

    fres = f_lseek(&fno, offset); //move file pointer by byte offset
    errorcheck_goto(fres, end_closefile);

    UINT bytesRead = 0;
    fres = f_read(&fno, output, numChars, &bytesRead); //read numChars bytes from file fno into output
end_closefile:
    fres_temp = f_close(&fno);
    keep_first_error(fres, fres_temp);
    return fres;
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Append data to file
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      filename - name of file to be written to (including filetype extension);
*                    data - string containing data to be written;
*                    size - number of bytes to write
* @param[output]     none
* @return            FRESULT of operation
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT SD_appendData(const char* filename, const void* data, UINT size)
{
    FIL fno; FRESULT fres, fres_temp;
    fres = f_open(&fno, filename, FA_WRITE | FA_OPEN_EXISTING); //open the file for reading
    errorcheck_ret(fres);

    fres = f_lseek(&fno, f_size(&fno)); //offset read/write pointer from 0 by the size of the file
    errorcheck_goto(fres, end_closefile);

    UINT bytesWritten = 0;
    fres = f_write(&fno, data, size, &bytesWritten); //write the data at the offset
    errorcheck_goto(fres, end_closefile);

    if (bytesWritten < size)
    {
        //ran out of space on SD card
        //Delete everything you just wrote so we don't leave partial data 
        fres = f_lseek(&fno, f_size(&fno) - bytesWritten); errorcheck_goto(fres, end_closefile);
        fres = f_truncate(&fno); errorcheck_goto(fres, end_closefile);
        // Raise a logic error so that the OBC restarts and cleans SD space on startup
        Sys_RaiseLogicError(LERR_WARN);
    }

end_closefile:
    fres_temp = f_close(&fno);
    keep_first_error(fres, fres_temp);
    return fres;
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Overwrite contents of file
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      filename - name of file to be written to (including filetype extension);
*                    data - string containing data to be written;
*                    size - number of bytes to write
* @param[output]     none
* @return            FRESULT of operation
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT SD_overwriteData(const char* filename, const void* data, UINT size)
{
    FIL fno; FRESULT fres, fres_temp;
    fres = f_open(&fno, filename, FA_WRITE | FA_OPEN_EXISTING); //open the file for reading
    errorcheck_ret(fres);

    // If new data is longer than what is in the file now,
    // try pre-allocating space before actually writing anything
    if (f_size(&fno) < size)
    {
        fres = f_lseek(&fno, size); errorcheck_goto(fres, end_closefile);
        if (f_tell(&fno) != size)
        {
            //ran out of space on SD card
            // Raise a logic error so that the OBC restarts and cleans SD space on startup
            Sys_RaiseLogicError(LERR_WARN);
        }
        fres = f_rewind(&fno); errorcheck_goto(fres, end_closefile);
    }

    UINT bytesWritten = 0;
    fres = f_write(&fno, data, size, &bytesWritten); //write the data at the start of the file
    errorcheck_goto(fres, end_closefile);

    fres = f_truncate(&fno); //delete any data remaining after the end of the newly written data

end_closefile:
    fres_temp = f_close(&fno);
    keep_first_error(fres, fres_temp);
    return fres;
}

#ifdef DEBUG_ENABLED
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* TEST DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static const Test_t Benchmark_Tests[] = {
    {.test = SD_Test_CreateFile_Basic, .testName = "Create file basic"},
    {.test = SD_Test_DeleteFile_Basic, .testName = "Delete file basic"},
    {.test = SD_Test_CreateDelFile_Advanced, .testName = "Advanced creation/deletion of files"},
    {.test = SD_Test_ManyFiles, .testName = "Creating then deleting many files"},
    {.test = SD_Test_AppendRead_Basic, .testName = "Appending and getting contents basic"},
    {.test = SD_Test_AppendRead_Twice, .testName = "Appending a second time"},
    {.test = SD_Test_ReadOffset, .testName = "Get contents with offset"},
    {.test = SD_Test_Overwrite, .testName = "Overwriting data"},
    {.test = SD_Test_GetFreeSpace, .testName = "Getting free space"},
};

DefineTestSet(SD_Routines_BenchmarkSet, SD_Routines, Benchmark_Tests);

#define TEST_FILE_NAME "DUMMYFIL"
_Static_assert((sizeof(TEST_FILE_NAME) + 3) / sizeof(char) <= SD_MAX_FILENAME, "test file name too long");

TestStatus_e SD_Test_Init()
{
    if (InitCdCard() == pdTRUE) return TEST_PASS;
    else return TEST_FAIL;
}

TestStatus_e SD_TestFixture_NoDummyFile()
{
    FRESULT fres = f_stat(TEST_FILE_NAME, NULL);
    if (fres == FR_NO_FILE) return TEST_PASS;
    else if (fres == FR_OK)
    {
        fres = f_unlink(TEST_FILE_NAME);
        test_assert(fres == FR_OK);
        return TEST_PASS;
    }
    else return TEST_FAIL;
}

TestStatus_e SD_Test_CreateFile_Basic()
{
    use_test_fixture(SD_TestFixture_NoDummyFile);

    FRESULT fres = SD_createFile(TEST_FILE_NAME);
    test_assert(fres == FR_OK);

    fres = f_stat(TEST_FILE_NAME, NULL);
    test_assert(fres == FR_OK);

    return TEST_PASS;
}

TestStatus_e SD_Test_DeleteFile_Basic()
{
    use_test_fixture(SD_TestFixture_NoDummyFile);

    use_test_fixture(SD_Test_CreateFile_Basic);

    FRESULT fres = SD_delFile(TEST_FILE_NAME);
    test_assert(fres == FR_OK);

    fres = f_stat(TEST_FILE_NAME, NULL);
    test_assert(fres == FR_NO_FILE);

    return TEST_PASS;
}

TestStatus_e SD_Test_CreateDelFile_Advanced()
{
    use_test_fixture(SD_TestFixture_NoDummyFile);
    FRESULT fres;

    // no file name
    fres = SD_createFile("");
    test_assert(fres == FR_INVALID_NAME);
    fres = SD_delFile("");
    test_assert(fres == FR_INVALID_NAME);

    // file name too long
    _Static_assert(sizeof("waaaytoooolongfilenameyeah") / sizeof(char) > SD_MAX_FILENAME, "long file name not long enough");
    fres = SD_createFile("waaaytoooolongfilenameyeah");
    test_assert(fres == FR_INVALID_NAME);

    // non-ASCII filename
    const char nastyName[] = { 0x1, INT8_MAX, 0x0 };
    fres = SD_createFile(nastyName);
    test_assert(fres == FR_INVALID_NAME);
    fres = SD_delFile(nastyName);
    test_assert(fres == FR_INVALID_NAME);

    // create duplicate file
    use_test_fixture(SD_Test_CreateFile_Basic);
    fres = SD_createFile(TEST_FILE_NAME);
    test_assert(fres == FR_EXIST);

    // delete non-existing file
    use_test_fixture(SD_TestFixture_NoDummyFile);
    fres = SD_delFile(TEST_FILE_NAME);
    test_assert(fres == FR_NO_FILE);

    // TODO: test null arguments

    return TEST_PASS;
}

#define LARGE_NUM_FILES 32

TestStatus_e SD_TestFixture_RemoveArtifacts()
{
    DIR dir; FILINFO fno; FRESULT fres;
    for (fres = f_findfirst(&dir, &fno, SD_ROOT, TEST_FILE_NAME "*");
        fres == FR_OK && fno.fname[0];
        fres = f_findnext(&dir, &fno))
    {
        fres = f_unlink(fno.fname);
        test_assert(fres == FR_OK);
    }
    return TEST_PASS;

}

TestStatus_e SD_Test_ManyFiles()
{
    char filename[sizeof(TEST_FILE_NAME) + 3] = TEST_FILE_NAME; FRESULT fres;

    for (int i = 0; i < LARGE_NUM_FILES; i++)
    {
        snprintf(filename + sizeof(TEST_FILE_NAME) - 1, 4, "%.3d", i);
        fres = SD_createFile(filename);
        if (fres != FR_OK)
        {
            use_test_fixture(SD_TestFixture_RemoveArtifacts);
            return TEST_FAIL;
        }
    }
    for (int i = 0; i < LARGE_NUM_FILES; i++)
    {
        snprintf(filename + sizeof(TEST_FILE_NAME) - 1, 4, "%.3d", i);
        fres = SD_delFile(filename);
        if (fres != FR_OK)
        {
            use_test_fixture(SD_TestFixture_RemoveArtifacts);
            return TEST_FAIL;
        }

    }
    return TEST_PASS;
}

#define TEST_DATA ("Hello World!")
#define TEST_DATA_LEN sizeof(TEST_DATA)

TestStatus_e SD_TestFixture_TestData()
{
    use_test_fixture(SD_TestFixture_NoDummyFile);

    FRESULT fres;

    fres = SD_createFile(TEST_FILE_NAME);
    test_assert(fres == FR_OK);

    fres = SD_appendData(TEST_FILE_NAME, TEST_DATA, TEST_DATA_LEN);
    test_assert(fres == FR_OK);

    return TEST_PASS;
}

TestStatus_e SD_CheckTestFileSz(FSIZE_t expectSz)
{
    FIL fd; FRESULT fres;
    fres = f_open(&fd, TEST_FILE_NAME, FA_READ | FA_OPEN_EXISTING);
    test_assert(fres == FR_OK);
    test_assert(f_size(&fd) == expectSz);
    f_close(&fd);
    test_assert(fres == FR_OK);
    return TEST_PASS;
}

TestStatus_e SD_Test_AppendRead_Basic()
{
    use_test_fixture(SD_TestFixture_TestData);

    test_assert(SD_CheckTestFileSz(TEST_DATA_LEN) == TEST_PASS);

    char buf[TEST_DATA_LEN]; FRESULT fres;
    fres = SD_getFileContents(TEST_FILE_NAME, buf, TEST_DATA_LEN);
    test_assert(fres == FR_OK);
    test_assert(memcmp(buf, TEST_DATA, TEST_DATA_LEN) == 0);

    use_test_fixture(SD_TestFixture_NoDummyFile);
    fres = SD_getFileContents(TEST_FILE_NAME, buf, TEST_DATA_LEN);
    test_assert(fres == FR_NO_FILE);

    return TEST_PASS;
}

TestStatus_e SD_Test_AppendRead_Twice()
{
    use_test_fixture(SD_TestFixture_TestData);
    FRESULT fres;
    
    // two consecutive copies of TEST_DATA 
    fres = SD_appendData(TEST_FILE_NAME, TEST_DATA, TEST_DATA_LEN);
    test_assert(fres == FR_OK);
    test_assert(SD_CheckTestFileSz(TEST_DATA_LEN*2) == TEST_PASS);

    char buf[TEST_DATA_LEN * 2];
    fres = SD_getFileContents(TEST_FILE_NAME, buf, TEST_DATA_LEN*2);
    test_assert(fres == FR_OK);
    test_assert(memcmp(buf, TEST_DATA, TEST_DATA_LEN) == 0);
    test_assert(memcmp(buf + TEST_DATA_LEN, TEST_DATA, TEST_DATA_LEN) == 0);

    return TEST_PASS;
}

TestStatus_e SD_Test_ReadOffset()
{
    use_test_fixture(SD_TestFixture_TestData);
    
    char buf[TEST_DATA_LEN / 2]; FRESULT fres;

    // read first half
    fres = SD_getFileContentsOffset(TEST_FILE_NAME, 0, buf, TEST_DATA_LEN / 2);
    test_assert(fres == FR_OK);
    test_assert(memcmp(buf, TEST_DATA, TEST_DATA_LEN / 2) == 0);

    // read second half
    fres = SD_getFileContentsOffset(TEST_FILE_NAME, TEST_DATA_LEN / 2, buf, TEST_DATA_LEN / 2);
    test_assert(fres == FR_OK);
    test_assert(memcmp(buf, &(TEST_DATA[TEST_DATA_LEN / 2]), TEST_DATA_LEN / 2) == 0);

    use_test_fixture(SD_TestFixture_NoDummyFile);
    fres = SD_getFileContentsOffset(TEST_FILE_NAME, TEST_DATA_LEN / 2, buf, TEST_DATA_LEN / 2);
    test_assert(fres == FR_NO_FILE);

    return TEST_PASS;
}

TestStatus_e SD_Test_Overwrite()
{
    use_test_fixture(SD_TestFixture_TestData);

    FRESULT fres;

    // Overwrite with shorter data
    char myShortData[] = "ha";

    fres = SD_overwriteData(TEST_FILE_NAME, myShortData, sizeof(myShortData));
    test_assert(fres == FR_OK);
    test_assert(SD_CheckTestFileSz(sizeof(myShortData)) == TEST_PASS);

    char bufShort[sizeof(myShortData)];
    fres = SD_getFileContents(TEST_FILE_NAME, bufShort, sizeof(myShortData));
    test_assert(fres == FR_OK);
    test_assert(memcmp(bufShort, myShortData, sizeof(myShortData)) == 0);

    // Overwrite with longer data
    char myLongData[] = "very long data string right here sheesh";

    fres = SD_overwriteData(TEST_FILE_NAME, myLongData, sizeof(myLongData));
    test_assert(fres == FR_OK);
    test_assert(SD_CheckTestFileSz(sizeof(myLongData)) == TEST_PASS);

    char bufLong[sizeof(myLongData)];
    fres = SD_getFileContents(TEST_FILE_NAME, bufLong, sizeof(myLongData));
    test_assert(fres == FR_OK);
    test_assert(memcmp(bufLong, myLongData, sizeof(myLongData)) == 0);

    use_test_fixture(SD_TestFixture_NoDummyFile);
    fres = SD_overwriteData(TEST_FILE_NAME, myLongData, sizeof(myLongData));
    test_assert(fres == FR_NO_FILE);

    return TEST_PASS;
}

TestStatus_e SD_Test_GetFreeSpace()
{
    // Roughly check that GetFreeSpace returns a sane number
    uint32_t kb;
    FRESULT fres = SD_getFreeSpace(&kb);
    test_assert(fres == FR_OK);
    // range (1.5 GB, 2.5 GB) since the SD card is 2 GB and should be basically empty
    test_assert(1.5 * 1000000 < kb && kb < 2.5 * 1000000);
    return TEST_PASS;
}

#endif
