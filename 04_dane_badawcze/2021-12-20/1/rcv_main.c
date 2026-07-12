/*
 * Author: Mariusz Krej
 *
 */

/**
 * @file
 * @brief Główny plik źródłowy testu TBD
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <SigGen.h>
#include <TrackLoop.h>

#include "GPS_L1_codes.h"

#include "my-math.h"
#include <BinMeas.h>
#include "MeasLogicGps.h"


/*
#define F_SAMPLE (32 * 1023000LL * 10)
#define F_SAMPLE (32 * 1023000LL)
#define F_SAMPLE (4 * 1023000LL)
#define F_SAMPLE (32800000LL)
*/
#define F_SAMPLE (4400000LL)


// częstotliwość pośrednia, trzeba też odpowienio zmieniać w gps-sdr-sim
/*
#define I_FREQ (0)
#define I_FREQ (50000)
 */
#define I_FREQ (1023000 / 4)


#define CHAN_NUM (7)

static int sats[] = { 2, 24, 6, 29, 14, 25, 12 };

// 2014/12/20,00:00:00
static int sig_freqs[] = { 2180, -2522, 33, 3679, -90, 2151, -330 };
// 2014/12/20,01:14:00
//static int sig_freqs[] = { -223, -3817, -2649, 2329, -2522, 342, -2142 };


#define SAVE_SIG_CSV (1)

//#define SIM_LEN   (F_SAMPLE * 4)
#define SIM_LEN   (0)

//#define SIG_FILE_PATH "../../../add_disk/GnsMeasData/gpssim_pw_4M_10s.8b"
#define SIG_FILE_PATH "pipe.8b"


#define CHIP_RATE (1023000)
#define CODE_LEN  (1023)


#define SEQ_RATE (CHIP_RATE / CODE_LEN)
#define SEQ_SAMP (F_SAMPLE / SEQ_RATE)

// dodatkowy szum oprócz tego który jest już w pliku wejściowym
#define NOISE_AMP (0)

// tego zależy od pliku wejściowego, gps-sdr-sim ma 128
#define SIG_AMP (128)

#define _SIGN(value) (value >= 0 ? 1 : -1)
// mnożenie Q10 i kwantowanie 1, 3, 5, 7 ...
#define _ADC_SCALE(value, mul_Q10) (1 + (((abs(value) * (mul_Q10 >> 1)) >> 10) << 1)) * _SIGN(value)

#define Q16 (0x80000000UL)


#if F_SAMPLE < 30000000
#define CHIP_DIV (2)
#else
#define CHIP_DIV (5)
//#define CHIP_DIV (2)
#endif


// liczba sekwencji integrowanych w trybie długiej integracji
#define LONG_INTEG_SEQ (10)


/// Random seed 1
static uint32_t randState1 = 1234567;

/// Random seed 2
static uint32_t randState2 = 5432167;


/**
 *
 * Wydruk wartości akumulatorów oraz filtrów.
 * @param loop pętla
 * @param fcsv wskaźnik pliku csv
 */
static void write_sig_csv(TrackLoop *loop, FILE *fcsv)
{
    fprintf(fcsv, "%6d; ", loop->seq_count - 1);
    fprintf(fcsv, "%6d; ", loop->channel);

    for (int i = 0; i < 5; i++)
    {
        fprintf(fcsv, "%6d; ", loop->veplv_accu_i_sav[i]);
        fprintf(fcsv, "%6d; ", loop->veplv_accu_q_sav[i]);
    }

    fprintf(fcsv, "%6d; ", loop->pll_disc_flt_sav);
    fprintf(fcsv, "%6d; ", loop->dll_disc_flt_sav);

    fprintf(fcsv, "\n");

}


/**
 * Zapis nagłówka pliku csv.
 *
 * @param fcsv wskaźnik pliku csv
 */
void save_sig_header(FILE* fcsv)
{
    fprintf(fcsv, "%7s", "idx;");
    fprintf(fcsv, "%7s", "CH;");
    fprintf(fcsv, "%8s", "IVE;");
    fprintf(fcsv, "%8s", "QVE;");
    fprintf(fcsv, "%8s", "IE;");
    fprintf(fcsv, "%8s", "QE;");
    fprintf(fcsv, "%8s", "IP;");
    fprintf(fcsv, "%8s", "QP;");
    fprintf(fcsv, "%8s", "IL;");
    fprintf(fcsv, "%8s", "QL;");
    fprintf(fcsv, "%8s", "IVL;");
    fprintf(fcsv, "%8s", "QVL;");
    fprintf(fcsv, "%8s", "PLL-Ft;");
    fprintf(fcsv, "%8s", "DLL-Ft;");
    fprintf(fcsv, "\n");
}


const int _pack_siz = 2048;
static FILE* _fsig;
int8_t* _read_buf;
int _read_buf_idx = 0;


int openSigFile()
{
    printf("reading file '%s' \n", SIG_FILE_PATH);

#ifndef _WIN32
    _fsig = fopen(SIG_FILE_PATH, "r");
#else
    _fsig = fopen(SIG_FILE_PATH, "rb");
#endif

    if (!_fsig)
    {
        printf("\nFILE OPEN ERROR !!!!\n");
        return 1;
    }
    _read_buf = (int8_t*)malloc(_pack_siz);
    _read_buf_idx = 0;

    return 0;
}


static int read_samp(int *sigI, int *sigQ)
{
    if(_read_buf_idx == 0)
    {
        int fileRd = fread(_read_buf, sizeof(int8_t), _pack_siz, _fsig);
        if(fileRd != _pack_siz)
        {
            if (sigI) *sigI = 1;
            if (sigQ) *sigQ = 1;
            return 0;
        }
    }
    int sampI, sampQ;

    //decodeIQ8b(inpBuf + _inpBufIdx, &sampI, &sampQ);
    sampI = _read_buf[_read_buf_idx + 0];
    sampQ = _read_buf[_read_buf_idx + 1];
    _read_buf_idx += 2;

    if (_read_buf_idx == _pack_siz)
        _read_buf_idx = 0;

    if (sigI) *sigI = sampI;
    if (sigQ) *sigQ = sampQ;

    return 1;
}


static int _dbg_samp_print = 30;

static int gen_samp(int *sigI, int *sigQ)
{
    if(!read_samp(sigI, sigQ))
        return 0;

    if (NOISE_AMP)
    {
        int32_t noise = (((runif(&randState1, &randState2) - Q16) << 1) * NOISE_AMP) >> 32;
        *sigI += noise;
        *sigQ += noise;
        *sigI = SIG_AMP * *sigI / (SIG_AMP + NOISE_AMP);
        *sigQ = SIG_AMP * *sigQ / (SIG_AMP + NOISE_AMP);
    }

    if (1)
    {
        *sigI = _ADC_SCALE(*sigI, 100);
        *sigQ = _ADC_SCALE(*sigQ, 100);
    }
    else
    {
        // kwantowanie 1-bit
        *sigI = 2 * (*sigI >= 0) - 1;
        *sigQ = 2 * (*sigQ >= 0) - 1;
    }

    if (_dbg_samp_print)
    {
        printf("(%d, %d) ", *sigI, *sigQ);
        _dbg_samp_print--;
        if (!_dbg_samp_print)
            printf("...\n");
    }

    return 1;
}


int get_sig_buf(int *acqI, int *acqQ, int len)
{
    int sigI, sigQ;
    for (int i = 0; i < len; ++i)
    {
        if (!gen_samp(&sigI, &sigQ))
            return 0;
        acqI[i] = sigI;
        acqQ[i] = sigQ;
    }
    return 1;
}


void pseudo_acq(int *acqI, int *acqQ, int acq_len, uint32_t pCodeL1[], int sig_freq, int *resofs)
{
    // pseudo i na szybko akwizycja
    // - nieoptymalna - bez wykorzystania wszystkich akumulatorów
    // - tylko w wymiarze kodu
    *resofs = 0;

    int64_t mxcorrsq = 0;
    int mxofs = 0;
    // krok skanowania - pół chipa (w próbkach)
    int step = F_SAMPLE / CHIP_RATE / 2;
    for (int ofs = 0; ofs < SEQ_SAMP; ofs += step)
    {
        TrackLoop track_mem;
        TrackLoop* track = TrackLoop_init(&track_mem, 0,
                CHIP_RATE, F_SAMPLE, sig_freq, pCodeL1, CODE_LEN, NULL, 0, 0, 2, 50, 50);
        track->freq_feedback_on = 0;
        track->code_feedback_on = 0;
        for (int i = 0; i < SEQ_SAMP; i++)
            TrackLoop_next(track, acqI[ofs + i], acqQ[ofs + i], 0);

        int64_t corrsq = SQR((int64_t)track->veplv_accu_i[2] + track->veplv_accu_i_sav[2]) +
                         SQR((int64_t)track->veplv_accu_q[2] + track->veplv_accu_q_sav[2]);

        if (corrsq > mxcorrsq)
        {
            if (0)
                printf("ofs: %5d corr: %5d \n", ofs, (int) sqrt(corrsq));
            mxcorrsq = corrsq;
            mxofs = ofs;
        }
    }
    printf("mxofs: %5d mxcorr: %5d  \n", mxofs, (int) sqrt(mxcorrsq));
    *resofs = mxofs;
}


static char _meas_file_buf[128];

FILE* open_meas_file()
{
    //_observablesFile = fopen("observables.dat", "wb");
    FILE *meas_file = fopen("meas.bo", "wb");
    if (meas_file == NULL)
    {
        printf("Observables File WRITE OPEN FAILED \n");
        return NULL;
    }
    setbuffer(meas_file, _meas_file_buf, sizeof(_meas_file_buf));
    FILE *finf = fopen("meas.info", "w");
    fprintf(finf, "CHAN_NUM = %d\n", MAX_MEAS_CHAN);
    fclose (finf);
    return meas_file;
}


typedef struct
{
    // prn > 0 => kanał aktywny
    int prn;

    // tu mógłby być docelowo zamiast TrackLoop typ HardChan lub 0 dla nieaktywnego
    TrackLoop track;

    DecodeL1 decoder;

    int long_integ;

    // akumulator cykli fazy
    int64_t carr_counter_accu;

    // poprzedni czas pomiaru
    int64_t prev_carr_meas_tm;

    // akumulator ułamków biasu których jescze nie usunięto
    int carr_bias_accu_q10;

} SoftChan;


SoftChan* SwChannel_init(SoftChan *self)
{
    self->prn = 0;
    self->long_integ = 0;
    self->carr_counter_accu = 0;
    self->prev_carr_meas_tm = 0;
    self->carr_bias_accu_q10 = 0;
    return self;
}


void chan_meas(SoftChan *chann, int chan_id, MeasLogicGps *measLogic, int64_t carr_meas_tm, int64_t tm_meas_freq, FILE *rgcsv)
{
    TrackLoop *track = &chann->track;
    int64_t seq_fract_q32;

    // hardware powinien dać zatrzaśnięty czas pomiaru
    // najlepiej żeby delta czasu była stała (licznik wyzwalający pomiar)
    // albo inna częstotliwość licznika czasu
    int carr_counter_upd;
    int32_t carr_ptr_q10;
    TrackLoop_meas(track, &seq_fract_q32, &carr_counter_upd, &carr_ptr_q10);

    // skomplikowane bo zerowanie samej części całkowitej licznika fazy wymieszane z biasem jest podchwytliwe
    int64_t bias_q10 = ((int64_t) I_FREQ << 10) * (carr_meas_tm - chann->prev_carr_meas_tm) / tm_meas_freq;
    chann->carr_bias_accu_q10 += bias_q10;
    chann->carr_counter_accu += carr_counter_upd;
    int to_rem = chann->carr_bias_accu_q10 >> 10;
    chann->carr_counter_accu -= to_rem;
    chann->carr_bias_accu_q10 -= to_rem << 10;
    int64_t curr_carr_q10 = (chann->carr_counter_accu << 10) + carr_ptr_q10 - chann->carr_bias_accu_q10;

    chann->prev_carr_meas_tm = carr_meas_tm;

    if (rgcsv && chan_id == 2)
        fprintf(rgcsv, "%d; %d; %d; %g; %d; %g; %.12g\n", track->seq_count - 1, chan_id, track->sum_seq_idx, (double) track->pcode_idx / track->pcode_len, track->chip_div_idx,
                (double) track->code_ptr / (1LL << 32), (double) seq_fract_q32 / (1LL << 32));

    //if (chan_id == 0)
    //    printf("chip = %.3f \n", (seqFractQ32 * 1023) / (double) (1LL << 32));

    MeasLogicGps_next_range(measLogic, chan_id, seq_fract_q32, -curr_carr_q10);
}

/**
 * Główna funkcja testu TBD
 *
 *
 * @return kod błędu, 0 to brak błędów
 */
int main()
{

    printf("F_SAMPLE = %lld\n", F_SAMPLE);
    printf("CHIP_DIV = %d\n", CHIP_DIV);
    printf("LONG_INTEG_SEQ = %d\n", LONG_INTEG_SEQ);
    printf("I_FREQ = %d\n", I_FREQ);

    printf("noiseAmp = %d \n", NOISE_AMP);

    openSigFile();

    int acq_len = SEQ_SAMP * 2;
    int acqI[acq_len];
    int acqQ[acq_len];

    if (!get_sig_buf(acqI, acqQ, acq_len))
    {
        printf("BŁĄD - brak próbek do akwizycji \n");
        return 1;
    }

    FILE *fsigcsv[CHAN_NUM];

    if (SAVE_SIG_CSV)
    {
        char nm_buf[500];
        for (int ch = 0; ch < CHAN_NUM; ++ch)
        {
            sprintf(nm_buf, "signal-ch-%d.csv", ch);
            printf("saving '%s' ... \n", nm_buf);
            fsigcsv[ch] = fopen(nm_buf, "w");
            save_sig_header(fsigcsv[ch]);
        }
    }

    int meas_interval = 100;
    MeasLogicGps measLogic_mem;
    MeasLogicGps *measLogic = MeasLogicGps_init(&measLogic_mem, CHAN_NUM, meas_interval, F_SAMPLE);

    printf("acquisition: \n");
    int64_t next_meas_idx = 0;

    static SoftChan channels[CHAN_NUM];
    for (int chan_id = 0; chan_id < CHAN_NUM; ++chan_id)
    {
        SoftChan *chann = channels + chan_id;
        SwChannel_init(chann);
        chann->prn = sats[chan_id];
        TrackLoop *track = TrackLoop_init(
            &chann->track,
            chan_id,
            CHIP_RATE, F_SAMPLE, I_FREQ + sig_freqs[chan_id],
            code_L1[chann->prn], CODE_LEN, NULL, 0, 0, CHIP_DIV, 25, 25);

        int band = 0;
        MeasLogicGps_chan_init(measLogic, chan_id, chann->prn, SEQ_RATE, band);
        DecodeL1* decoder = DecodeL1_init(&chann->decoder, chann->prn);
        MeasLogicGps_chan_set_decoder(measLogic, chan_id, decoder);

        int offset;
        pseudo_acq(acqI, acqQ, acq_len, code_L1[chann->prn], I_FREQ + sig_freqs[chan_id], &offset);

        printf("offset %d (samp)\n", offset);

        int seqEnd;
        for (int i = 0; i < SEQ_SAMP - 1 - offset; ++i)
            TrackLoop_next(track, 0, 0 , &seqEnd);
    }

    FILE *meas_file = open_meas_file();

    //todo to osobno w kanałach
    int filters_state_interv = F_SAMPLE * 3;
    int next_flt_chng = filters_state_interv;
    // odwrotna kolejność nastawiania
    //int pll_bn_steps[] = { 16, 30, 50};
    //int dll_bn_steps[] = { 2,  18, 50};
    float pll_bn_steps[] = { 4,   8, 15, 25};
    float dll_bn_steps[] = { 0.5, 1,  9, 25};

    int flt_step = 3;

    FILE *rgcsv = 0;
    if (0)
    {
        rgcsv = fopen("rg-test-ch2.csv", "w");
        fprintf(rgcsv, "seq_count; chan_id; sum_seq_idx; pcode_idx; chip_div_idx; code_ptr; seqFract\n");
    }

    printf("tracking: \n");

    int64_t isamp = 0;
    while (++isamp)
    {
        int sigI, sigQ;
        if (!gen_samp(&sigI, &sigQ))
            break;

        for (int chan_id = 0; chan_id < CHAN_NUM; ++chan_id)
        {
            SoftChan *chann = channels + chan_id;
            if (chann->prn <= 0)
                continue;
            TrackLoop *track = &chann->track;

            int dump;
            TrackLoop_next(track, sigI, sigQ, &dump);

            if (dump)
            {
                int frameFound;
                int ip = track->veplv_accu_i_sav[2];
                int nip = !chann->long_integ ? 1 : LONG_INTEG_SEQ;
                for (int ii = 0; ii < nip; ++ii)
                    MeasLogicGps_next_seq(measLogic, chan_id, ip, &frameFound);
                if (SAVE_SIG_CSV)
                    write_sig_csv(track, fsigcsv[chan_id]);

                // TODO tu powinien być czas po przestawieniu filtrów
                if (LONG_INTEG_SEQ > 1 && !chann->long_integ && isamp > F_SAMPLE * 7LL)
                {
                    MeasLogicGpsChan *mchan = measLogic->channels + chan_id;
                    if (mchan->msg_fnd)
                    {
                        int bit_seq_idx = mchan->tow_seqs % 20;
                        if ((bit_seq_idx % LONG_INTEG_SEQ) == 0)
                        {
                            chann->long_integ = 1;
                            track->sum_seqs = LONG_INTEG_SEQ;
                            track->sum_seq_idx = (bit_seq_idx + 0) % LONG_INTEG_SEQ;
                            int pll_bn = pll_bn_steps[flt_step];
                            int dll_bn = dll_bn_steps[flt_step];
                            printf("** SAT %d set long integ\n", chann->prn);
                            TrackLoop_set_filters(track, CHIP_RATE, pll_bn, dll_bn);
                        }
                    }
                }

            }
        }

        if (isamp >= next_meas_idx)
        {
            next_meas_idx = isamp + F_SAMPLE / 20;
            MeasLogicGps_start_range(measLogic, isamp);

            for (int chan_id = 0; chan_id < CHAN_NUM; ++chan_id)
            {
                chan_meas(channels + chan_id, chan_id, measLogic, isamp, F_SAMPLE, rgcsv);
            }

            if (MeasLogicGps_meas(measLogic))
            {
                BinMeas_fwrite(meas_file, MeasLogicGps_last_meas(measLogic), MAX_MEAS_CHAN);
            }
        }

        if (flt_step && isamp >= next_flt_chng)
        {
            next_flt_chng = isamp + filters_state_interv;
            flt_step--;
            for (int chan_id = 0; chan_id < CHAN_NUM; ++chan_id)
            {
                SoftChan *chann = channels + chan_id;
                TrackLoop *track = &chann->track;
                float pll_bn = pll_bn_steps[flt_step];
                float dll_bn = dll_bn_steps[flt_step];
                printf("** set filters PLL: %g DLL: %g\n", pll_bn, dll_bn);
                TrackLoop_set_filters(track, CHIP_RATE, pll_bn, dll_bn);
            }
        }

        if (0 && isamp == F_SAMPLE * 10)
            isamp = 0;

    }

    if (SAVE_SIG_CSV)
    {
        for (int ch = 0; ch < CHAN_NUM; ++ch)
        {
            fclose(fsigcsv[ch]);
            fsigcsv[ch] = 0;
        }
    }

    printf("...saved\n");

    if (rgcsv)
        fclose(rgcsv);
    rgcsv = 0;

    fclose(_fsig);
    _fsig = 0;

    fclose(meas_file);
    meas_file = 0;

    printf("\nDONE\n");
    return 0;
}
