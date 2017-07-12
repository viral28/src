/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Anton Deguet, Martin Kelly
  Created on: 2011-03-07

  (C) Copyright 2011 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsCMUSphinx4QtWidget_h
#define _mtsCMUSphinx4QtWidget_h

#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

// Always include last!
#include <sawCMUSphinx4/mtsCMUSphinx4ExportQt.h>
class CISST_EXPORT mtsCMUSphinx4QtWidget : public QWidget
{
    Q_OBJECT;

 public:
    mtsCMUSphinx4QtWidget(void);
    ~mtsCMUSphinx4QtWidget(void) {};

    std::multimap<QString,QString> ContextMap;

    QGridLayout * CentralLayout;
    QLabel * LabelContext;
    QLabel * ValueContext;
    QLabel * LabelWordRecognized;
    QLabel * ValueWordRecognized;
    QLabel * LabelVocabulary;
    QLabel * ValueVocabulary;
    QComboBox * WordSelector;
    QPushButton * TriggerButton;

 signals:
    void WordTriggered(QString);

 public slots:
    void AddWord(QString context, QString word);
    void ContextChanged(QString context);
    void GetTriggeredWord(void);

};

#endif  // _mtsCMUSphinx4QtWidget_h
