      SUBROUTINE XERRWV(MESSG,NMESSG,NERR,LEVEL,NI,I1,I2,NR,R1,R2)      XER   10
C
C     ABSTRACT
C        XERRWV PROCESSES A DIAGNOSTIC MESSAGE, IN A MANNER
C        DETERMINED BY THE VALUE OF LEVEL AND THE CURRENT VALUE
C        OF THE LIBRARY ERROR CONTROL FLAG, KONTRL.
C        (SEE SUBROUTINE XSETF FOR DETAILS.)
C        IN ADDITION, UP TO TWO INTEGER VALUES AND TWO REAL
C        VALUES MAY BE PRINTED ALONG WITH THE MESSAGE.
C
C     DESCRIPTION OF PARAMETERS
C      --INPUT--
C        MESSG - THE HOLLERITH MESSAGE TO BE PROCESSED.
C        NMESSG- THE ACTUAL NUMBER OF CHARACTERS IN MESSG.
C        NERR  - THE ERROR NUMBER ASSOCIATED WITH THIS MESSAGE.
C                NERR MUST NOT BE ZERO.
C        LEVEL - ERROR CATEGORY.
C                =2 MEANS THIS IS AN UNCONDITIONALLY FATAL ERROR.
C                =1 MEANS THIS IS A RECOVERABLE ERROR.  (I.E., IT IS
C                   NON-FATAL IF XSETF HAS BEEN APPROPRIATELY CALLED.)
C                =0 MEANS THIS IS A WARNING MESSAGE ONLY.
C                =-1 MEANS THIS IS A WARNING MESSAGE WHICH IS TO BE
C                   PRINTED AT MOST ONCE, REGARDLESS OF HOW MANY
C                   TIMES THIS CALL IS EXECUTED.
C        NI    - NUMBER OF INTEGER VALUES TO BE PRINTED. (O TO 2)
C        I1    - FIRST INTEGER VALUE.
C        I2    - SECOND INTEGER VALUE.
C        NR    - NUMBER OF REAL VALUES TO BE PRINTED. (0 TO 2)
C        R1    - FIRST REAL VALUE.
C        R2    - SECOND REAL VALUE.
C
C     EXAMPLES
C        CALL XERRWV(29HSMOOTH -- NUM (=I1) WAS ZERO.,29,1,2,
C    1   1,NUM,0,0,0.,0.)
C        CALL XERRWV(54HQUADXY -- REQUESTED ERROR (R1) LESS THAN MINIMUM
C    1 (R2).,54,77,1,0,0,0,2,ERRREQ,ERRMIN)
C
C     WRITTEN BY RON JONES, WITH SLATEC COMMON MATH LIBRARY SUBCOMMITTEE
C END OF ABSTRACT
C     LATEST REVISION ---  19 MAR 1980
C     REVISED BY K HASKELL TO CHECK INPUT ARGS, 2/18/80
C
      DIMENSION MESSG(NMESSG),LUN(5)
C     GET FLAGS
      LKNTRL = J4SAVE(2,0,.FALSE.)
      MAXMES = J4SAVE(4,0,.FALSE.)
C     CHECK FOR VALID INPUT
      IF (NMESSG.GT.0) GO TO 2
         IF (LKNTRL.GT.0) CALL XERPRT(17HFATAL ERROR IN...,17)
      CALL XERPRT (33HXERRWV -- NMESSG MUST BE POSITIVE,33)
      IF (LKNTRL.GT.0) CALL FDUMP
      IF (LKNTRL.GT.0) CALL XERPRT(29HJOB ABORT DUE TO FATAL ERROR.,
     1 29)
      IF (LKNTRL.GT.0) CALL XERSAV(1H ,0,0,0,KDUMMY)
      CALL XERABT (23HXERRWV -- INVALID INPUT,23)
      RETURN
    2 CONTINUE
      IF (NERR.NE.0) GO TO 4
      IF (LKNTRL.GT.0) CALL XERPRT(17HFATAL ERROR IN...,17)
      CALL XERPRT (28HXERRWV -- NERR=0 IS AN ERROR,28)
      IF (LKNTRL.GT.0) CALL FDUMP
      IF (LKNTRL.GT.0) CALL XERPRT(29HJOB ABORT DUE TO FATAL ERROR.,
     1 29)
      IF (LKNTRL.GT.0) CALL XERSAV(1H ,0,0,0,KDUMMY)
      CALL XERABT (23HXERRWV -- INVALID INPUT,23)
      RETURN
    4 CONTINUE
      IF ((LEVEL.GE.(-1)).AND.(LEVEL.LE.2)) GO TO 10
      IF (LKNTRL.GT.0) CALL XERPRT(17HFATAL ERROR IN...,17)
      CALL XERPRT (32HXERRWV -- INVALID VALUE OF LEVEL,32)
         IF (LKNTRL.GT.0) CALL FDUMP
         IF (LKNTRL.GT.0) CALL XERPRT(29HJOB ABORT DUE TO FATAL ERROR.,
     1   29)
         IF (LKNTRL.GT.0) CALL XERSAV(1H ,0,0,0,KDUMMY)
         CALL XERABT(23HXERROR -- INVALID INPUT,23)
         RETURN
   10 CONTINUE
C     RECORD MESSAGE
      JUNK = J4SAVE(1,NERR,.TRUE.)
      CALL XERSAV(MESSG,NMESSG,NERR,LEVEL,KOUNT)
C     LET USER OVERRIDE
      LFIRST = MESSG(1)
      LMESSG = NMESSG
      LERR = NERR
      LLEVEL = LEVEL
      CALL XERCTL(LFIRST,LMESSG,LERR,LLEVEL,LKNTRL)
C     RESET TO ORIGINAL VALUES
      LMESSG = NMESSG
      LERR = NERR
      LLEVEL = LEVEL
      LKNTRL = MAX0(-2,MIN0(2,LKNTRL))
      MKNTRL = IABS(LKNTRL)
C     DECIDE WHETHER TO PRINT MESSAGE
      IF ((LLEVEL.LT.2).AND.(LKNTRL.EQ.0)) GO TO 100
      IF (((LLEVEL.EQ.(-1)).AND.(KOUNT.GT.MIN0(1,MAXMES)))
     1.OR.((LLEVEL.EQ.0)   .AND.(KOUNT.GT.MAXMES))
     2.OR.((LLEVEL.EQ.1)   .AND.(KOUNT.GT.MAXMES).AND.(MKNTRL.EQ.1))
     3.OR.((LLEVEL.EQ.2)   .AND.(KOUNT.GT.MAX0(1,MAXMES)))) GO TO 100
         IF (LKNTRL.LE.0) GO TO 20
            CALL XERPRT(1H ,1)
C           INTRODUCTION
            IF (LLEVEL.EQ.(-1)) CALL XERPRT
     1(57HWARNING MESSAGE...THIS MESSAGE WILL ONLY BE PRINTED ONCE.,57)
            IF (LLEVEL.EQ.0) CALL XERPRT(13HWARNING IN...,13)
            IF (LLEVEL.EQ.1) CALL XERPRT
     1      (23HRECOVERABLE ERROR IN...,23)
            IF (LLEVEL.EQ.2) CALL XERPRT(17HFATAL ERROR IN...,17)
   20    CONTINUE
C        MESSAGE
         CALL XERPRT(MESSG,LMESSG)
         CALL XGETUA(LUN,NUNIT)
         DO 50 KUNIT=1,NUNIT
            IUNIT = LUN(KUNIT)
            IF (IUNIT.EQ.0) IUNIT = I1MACH(4)
            IF (NI.GE.1) WRITE (IUNIT,22) I1
            IF (NI.GE.2) WRITE (IUNIT,23) I2
            IF (NR.GE.1) WRITE (IUNIT,24) R1
            IF (NR.GE.2) WRITE (IUNIT,25) R2
   22       FORMAT (11X,21HIN ABOVE MESSAGE, I1=,I10)
   23       FORMAT (11X,21HIN ABOVE MESSAGE, I2=,I10)
   24       FORMAT (11X,21HIN ABOVE MESSAGE, R1=,E20.10)
   25       FORMAT (11X,21HIN ABOVE MESSAGE, R2=,E20.10)
            IF (LKNTRL.LE.0) GO TO 40
C              ERROR NUMBER
               WRITE (IUNIT,30) LERR
   30          FORMAT (15H ERROR NUMBER =,I10)
   40       CONTINUE
   50    CONTINUE
C        TRACE-BACK
         CALL FDUMP
  100 CONTINUE
      IFATAL = 0
      IF ((LLEVEL.EQ.2).OR.((LLEVEL.EQ.1).AND.(MKNTRL.EQ.2)))
     1IFATAL = 1
C     QUIT HERE IF MESSAGE IS NOT FATAL
      IF (IFATAL.LE.0) RETURN
      IF (LKNTRL.LE.0) GO TO 120
C        PRINT REASON FOR ABORT
         IF (LLEVEL.EQ.1) CALL XERPRT
     1   (35HJOB ABORT DUE TO UNRECOVERED ERROR.,35)
         IF (LLEVEL.EQ.2) CALL XERPRT
     1   (29HJOB ABORT DUE TO FATAL ERROR.,29)
C        PRINT ERROR SUMMARY
         CALL XERSAV(1H ,0,0,0,KDUMMY)
  120 CONTINUE
C     ABORT
      IF ((LLEVEL.EQ.2).AND.(KOUNT.GT.MAX0(1,MAXMES))) LMESSG = 0
      CALL XERABT(MESSG,LMESSG)
      RETURN
      END
