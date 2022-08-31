/*音声回りのファイルやファイル名確認*/

#include <BMI160Gen.h>
#include <MemoryUtil.h>
#include <SensorManager.h>
#include <AccelSensor.h>
#include <Aesm.h>
#include <ApplicationSensor.h>

#include <SDHCI.h>
#include <Audio.h>

//#define RECORD_FILE_NAME "Sound.mp3"
#define RecordLoopNum 800

SDClass theSD;
AudioClass *theAudio;

File myFile;


const int i2c_addr                = 0x68;
const int baudrate                = 115200;
const int walking_stride          = 60; /* 60cm */
const int running_stride          = 80; /* 80cm */
const int accel_range             =  2; /* 2G */
const int accel_rate              = 50; /* 50 Hz */
const int accel_sample_num        = 50; /* 50 sample */
const int accel_sample_size       = sizeof(float) * 3;

/*
static const int32_t recoding_time = 5;
static const int32_t recoding_bitrate = 96000;
static const int32_t recoding_byte_per_second = (recoding_bitrate / 8);
static const int32_t recoding_size = recoding_byte_per_second * recoding_time;
*/

enum
{
  PlayReady = 0,
  Playing,
  RecordReady,
  Recording
};

bool ErrEnd = false;

bool step_counter_result(sensor_command_data_mh_t &data)
{
  /* Display result of sensing */

  StepCounterStepInfo* steps =
              reinterpret_cast<StepCounterStepInfo*>
              (StepCountReader.subscribe(data));
  
  if (steps == NULL)
    {
      return 0;
    }

  float  tempo = 0;

  switch (steps->movement_type)
    {
      case STEP_COUNTER_MOVEMENT_TYPE_WALK:
      case STEP_COUNTER_MOVEMENT_TYPE_RUN:
        tempo = steps->tempo;
        break;
      case STEP_COUNTER_MOVEMENT_TYPE_STILL:

        /* In this state, the tempo value on the display is zero. */

        tempo = 0;

        break;
      default:

        /* It is not displayed in the state other than the above. */

        return 0;
    }

  printf("%11.5f,%11.2f,%11.5f,%11.5f,%11ld,",
               tempo,
               steps->stride,
               steps->speed,
               steps->distance,
               steps->step);

  switch (steps->movement_type)
    {
      case STEP_COUNTER_MOVEMENT_TYPE_STILL:
        puts("   stopping");
        break;
      case STEP_COUNTER_MOVEMENT_TYPE_WALK:
        puts("   walking");
        break;
      case STEP_COUNTER_MOVEMENT_TYPE_RUN:
        puts("   running");
        break;
      default:
        puts("   UNKNOWN");
        break;
    }
  return 0;
}

static void audio_attention_cb(const ErrorAttentionParam *atprm)
{
  puts("Attention!");
  
  if (atprm->error_code >= AS_ATTENTION_CODE_WARNING)
    {
      ErrEnd = true;
   }
}

void setup()
{
  Serial.begin(baudrate);
  while (!Serial);
  puts("1");

  
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  BMI160.setAccelerometerRange(accel_range);
  BMI160.setAccelerometerRate(accel_rate);

puts("2");    // for DEBUG
/*

  SensorManager.begin();

puts("3");    // for DEBUG


  AccelSensor.begin(SEN_accelID,
                    accel_rate,
                    accel_sample_num,
                    accel_sample_size);   //加速度センサクライアントの生成

puts("4");    // for DEBUG


  Aesm.begin(SEN_stepcounterID,
             SUBSCRIPTION(SEN_accelID),
             accel_rate,
             accel_sample_num,
             accel_sample_size);    //StepCounterデータを読み出すためのクライアント
  StepCountReader.begin(SEN_app0ID,
                        SUBSCRIPTION(SEN_stepcounterID),
                        step_counter_result);   //StepCounterライブラリを読み出すため
  Aesm.set(walking_stride, running_stride);   //歩幅のセット
*/
puts("3");    // for DEBUG

  puts("Start sensing...");
  puts("-----------------------------------------------------------------------");
  puts("      tempo,     stride,      speed,   distance,       step,  move-type");

  /* Initialize SD */
  while (!theSD.begin())
    {
      /* wait until SD card is mounted. */
      Serial.println("Insert SD card.");
    }

puts("4");    // for DEBUG

  theAudio = AudioClass::getInstance();

puts("5");    // for DEBUG

  theAudio->begin(audio_attention_cb);

//  puts("initialization Audio Library");

puts("6");    // for DEBUG

  theAudio->setRecorderMode(AS_SETRECDR_STS_INPUTDEVICE_MIC);

puts("7");    // for DEBUG

/*
  theAudio->initRecorder(AS_CODECTYPE_MP3, "/mnt/sd0/BIN", AS_SAMPLINGRATE_48000, AS_CHANNEL_STEREO);
*/

//  theAudio->initRecorder(AS_CODECTYPE_MP3, "/mnt/sd0/BIN", AS_SAMPLINGRATE_32000, AS_CHANNEL_STEREO);
  puts("Init Recorder!");

/*****************************************/
  SensorManager.begin();

  AccelSensor.begin(SEN_accelID,
                    accel_rate,
                    accel_sample_num,
                    accel_sample_size);   //加速度センサクライアントの生成

puts("8");    // for DEBUG


  Aesm.begin(SEN_stepcounterID,
             SUBSCRIPTION(SEN_accelID),
             accel_rate,
             accel_sample_num,
             accel_sample_size);    //StepCounterデータを読み出すためのクライアント
  StepCountReader.begin(SEN_app0ID,
                        SUBSCRIPTION(SEN_stepcounterID),
                        step_counter_result);   //StepCounterライブラリを読み出すため
  Aesm.set(walking_stride, running_stride);   //歩幅のセット

/**********************************************/

/*
  if (theSD.exists(RECORD_FILE_NAME))
    {
      printf("Remove existing file [%s].\n", RECORD_FILE_NAME);
      theSD.remove(RECORD_FILE_NAME);
    }

  myFile = theSD.open(RECORD_FILE_NAME, FILE_WRITE);
  /* Verify file open */
/*  if (!myFile)
    {
      printf("File open error\n");
      exit(1);
    }

  printf("Open! [%s]\n", RECORD_FILE_NAME);

  theAudio->startRecorder();
  puts("Recording Start!");
*/
}

void recorderMode(char *fname)
{
  /* Select input device as analog microphone */
  theAudio->setRecorderMode(AS_SETRECDR_STS_INPUTDEVICE_MIC);

  /*
   * Initialize filetype to stereo mp3 with 48 Kb/s sampling rate
   * Search for MP3 codec in "/mnt/sd0/BIN" directory
   */
  theAudio->initRecorder(AS_CODECTYPE_MP3, "/mnt/sd0/BIN", AS_SAMPLINGRATE_48000, AS_CHANNEL_STEREO);

  /* Open file for data write on SD card */
  myFile = theSD.open(fname, FILE_WRITE);
  /* Verify file open */
  if (!myFile)
    {
      printf("File open error\n");
      exit(1);
    }
  puts("Open!");

  puts("Rec!");

  theAudio->startRecorder();
}

bool recordStream()
{
  static int cnt = 0;
  err_t err = AUDIOLIB_ECODE_OK;

  /* recording end condition */
  if (cnt > RecordLoopNum)
    {
      puts("End Recording");
      goto stop_recorder;
    }

  /* Read frames to record in file */
  err = theAudio->readFrames(myFile);

  if (err != AUDIOLIB_ECODE_OK)
    {
      printf("File End! =%d\n",err);
      goto stop_recorder;
    }

  /* This sleep is adjusted by the time to write the audio stream file.
   * Please adjust in according with the processing contents
   * being processed at the same time by Application.
   *
   * The usleep() function suspends execution of the calling thread for usec
   * microseconds. But the timer resolution depends on the OS system tick time
   * which is 10 milliseconds (10,000 microseconds) by default. Therefore,
   * it will sleep for a longer time than the time requested here.
   */

  usleep(10000);
  cnt++;

  /* return still recording */
  return false;

stop_recorder:
  sleep(1);
  theAudio->stopRecorder();
  theAudio->closeOutputFile(myFile);
  myFile.close();
  cnt = 0;

  /* return record end */
  return true;
}

void loop()
{
//puts("a");    // for DEBUG

  float x, y, z;

  static int s_state = RecordReady;
  static int fcnt = 0;
  char fname[16] = "RecPlay";

  BMI160.readAccelerometerScaled(x, y, z);
//  AccelSensor.write_data(x, y, z);

/*********************************************/
  if (ErrEnd)
    {
      printf("Error End\n");
      exit(1);
    }

  switch (s_state)
    {        
      case RecordReady:
        snprintf(fname, sizeof(fname), "RecPlay%d.mp3", fcnt);
        recorderMode(fname);
        s_state = Recording;
        break;
        
      case Recording:
        if (recordStream())
          {
            theAudio->setReadyMode();
            s_state = RecordReady;
//            s_state = PlayReady;
          }
        break;

      default:
        break;
    }
/**********************************************/

//  err_t err;
  /* recording end condition */
 /*
  if (theAudio->getRecordingSize() > recoding_size)
    {
      theAudio->stopRecorder();
      sleep(1);
      err = theAudio->readFrames(myFile);

      goto exitRecording;
    }
*/
  /* Read frames to record in file */
/*
  err = theAudio->readFrames(myFile);

  if (err != AUDIOLIB_ECODE_OK)
    {
      printf("File End! =%d\n",err);
      theAudio->stopRecorder();
      goto exitRecording;
    }

  if (ErrEnd)
    {
      printf("Error End\n");
      theAudio->stopRecorder();
      goto exitRecording;
    }

  return;

exitRecording:

  theAudio->closeOutputFile(myFile);
  myFile.close();
  
  theAudio->setReadyMode();
  theAudio->end();
  
  puts("End Recording");
  exit(1);
*/
}