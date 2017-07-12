/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Martin Kelly, Anton Deguet
  Created on: 2011-02-15

  (C) Copyright 2011 Johns Hopkins University (JHU), All Rights
  Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#include <fstream>
#include <iostream>
#include <sstream>
#include <jni.h>
#include <stdlib.h>
#include <string.h>

#include <sawCMUSphinx4/mtsCMUSphinx4.h>

#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

CMN_IMPLEMENT_SERVICES(mtsCMUSphinx4);

struct mtsCMUSphinx4Java
{
    JNIEnv * Environment;
    JavaVM * VirtualMachine;
    jobject CMUSphinx4Wrapper;
    jmethodID SetCurrentContextMethod;
};


mtsCMUSphinx4::mtsCMUSphinx4(const std::string & componentName):
    mtsTaskContinuous(componentName),
    CurrentContext(0),
    MicrophoneNumber(1),
    NewWordRecognized(false)
{
    // own named map of contexts
    Contexts.SetOwner(*this);

    // allocate java data
    this->JavaData = new mtsCMUSphinx4Java;
    this->JavaData->Environment = 0;
    this->JavaData->VirtualMachine = 0;

    // defaults
    ModelTopDir = "WSJ_8gau_13dCep_16k_40mel_130Hz_6800Hz";
    ModelFile = std::string(SAW_SPHINX4_DIST_DIR) + "/lib/" + ModelTopDir + ".jar";
    SphinxFile = std::string(SAW_SPHINX4_DIST_DIR) + "/lib/sphinx4.jar";
    TemplateFile = std::string(SAW_SPHINX4_TEMPLATE_DIR) + "/sawCMUSphinx4.xml.template";
    SphinxWrapperDir = std::string(SAW_SPHINX4_JAVACLASS_DIR);
    ConfigName = "sawCMUSphinx4.config.xml";

    // trigger for any word received
    WordRecognizedTrigger = new mtsMulticastCommandWrite<mtsStdString>("WordRecognized", mtsStdString());
    NoWordRecognizedTrigger = new mtsMulticastCommandVoid("NoWordRecognized");

    // add default interface
    mtsInterfaceProvided * interfaceProvided = this->AddInterfaceProvided("Default");
    if (interfaceProvided) {
        // query list of contexts and words per context
        interfaceProvided->AddCommandRead(&mtsCMUSphinx4::GetContexts, this, "GetContexts");
        interfaceProvided->AddCommandQualifiedRead(&mtsCMUSphinx4::GetContextWords, this, "GetContextWords");
        // context changed
        interfaceProvided->AddEventWrite(this->ContextChangedTrigger, "ContextChanged", mtsStdString());
        // get word from UI
        interfaceProvided->AddCommandWrite(&mtsCMUSphinx4::WordTriggeredFromUI, this, "TriggerWordFromUI");
        // word recognized
        mtsCommandWriteBase * eventWriteTrigger = interfaceProvided->AddEventWrite("WordRecognized", mtsStdString());
        if (eventWriteTrigger) {
            WordRecognizedTrigger->AddCommand(eventWriteTrigger);
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "constructor: failed to add event \"WordRecognized\"" << std::endl;
        }
        // empty word
        mtsCommandVoid * eventVoidTrigger = interfaceProvided->AddEventVoid("NoWordRecognized");
        if (eventVoidTrigger) {
            NoWordRecognizedTrigger->AddCommand(eventVoidTrigger);
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "constructor: failed to add event \"NoWordRecognized\"" << std::endl;
        }
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "constructor: failed to add interface provided \"Default\"" << std::endl;
    }
}


mtsCMUSphinx4::Context * mtsCMUSphinx4::AddContext(const std::string & contextName)
{
    Context * context = new Context(contextName);
    if (context == 0) {
        CMN_LOG_CLASS_INIT_ERROR << "AddContext: failed to create new context" << std::endl;
        return 0;
    }
    if (Contexts.AddItem(context->GetName(), context)) {
        context->CMUSphinx4Wrapper = this;
        CMN_LOG_CLASS_INIT_DEBUG << "AddContext: added context \"" << context->GetName() << "\"" << std::endl;
        return context;
    }
    CMN_LOG_CLASS_INIT_ERROR << "AddContext: failed to add context \"" << context->GetName() << "\"" << std::endl;
    delete context;
    return 0;
}


bool mtsCMUSphinx4::SetCurrentContext(const std::string & contextName)
{
    Context * context = this->Contexts.GetItem(contextName);
    if (context) {
        CMN_LOG_CLASS_RUN_DEBUG << "SetCurrentContext: switching to context \"" << contextName << "\"" << std::endl;
        CurrentContext = context;
        // send starting context if the Java environment is already set
        if (this->JavaData->Environment) {
            jstring currentContext;
            currentContext = this->JavaData->Environment->NewStringUTF(contextName.c_str());
            this->JavaData->Environment->CallVoidMethod(this->JavaData->CMUSphinx4Wrapper,
                                                        this->JavaData->SetCurrentContextMethod,
                                                        currentContext);
            this->JavaData->Environment->ReleaseStringUTFChars(currentContext,
                                                               this->JavaData->Environment->GetStringUTFChars(currentContext, JNI_FALSE));
        }
        // trigger event
        this->ContextChangedTrigger(mtsStdString(contextName));
        return true;
    }
    CMN_LOG_CLASS_RUN_ERROR << "SetCurrentContext: can't find context \"" << contextName << "\"" << std::endl;
    return false;
}


bool mtsCMUSphinx4::SetCurrentContext(const mtsCMUSphinx4::Context * context)
{
    if (context) {
        return this->SetCurrentContext(context->GetName());
    }
    CMN_LOG_CLASS_RUN_ERROR << "SetCurrentContext: null pointer on context" << std::endl;
    return 0;
}


void mtsCMUSphinx4::GetContexts(stdStringVec & placeHolder) const
{
    ContextMap::const_iterator iterator;
    ContextMap::const_iterator end = Contexts.end();
    for (iterator = Contexts.begin();
         iterator != end;
         iterator++) {
        placeHolder.push_back(iterator->first);
    }
}


void mtsCMUSphinx4::GetContextWords(const mtsStdString & contextName,
                                          stdStringVec & placeHolder) const
{
    Context * context = this->Contexts.GetItem(contextName);
    if (context) {
        placeHolder = context->GetVocabulary();
    }
}


void mtsCMUSphinx4::WordTriggeredFromUI(const mtsStdString & word)
{
    WordRecognizedCallback(word);
}


unsigned int mtsCMUSphinx4::GetMicrophoneNumber(void) const
{
    return this->MicrophoneNumber;
}


void mtsCMUSphinx4::SetMicrophoneNumber(unsigned int microphoneNumber)
{
    this->MicrophoneNumber = microphoneNumber;
}


void mtsCMUSphinx4::PrintAudioDevices(void)
{

}


bool mtsCMUSphinx4::ReplaceAll(std::string & base,
                            const std::string & s,
                            const std::string & t)
{
    try {
        size_t sLength = s.length();
        size_t tLength = t.length();
        size_t foundPosition = base.find(s);
        while (foundPosition != std::string::npos) {
            base.replace(foundPosition, sLength, t);
            foundPosition = base.find(s, foundPosition + tLength);
        }
    } catch (std::exception & except) {
        CMN_LOG_CLASS_RUN_ERROR << "ReplaceAll: got exception " << except.what() << std::endl;
        return false;
    }
    return true;
}


bool mtsCMUSphinx4::StartJava(void)
{
    jint result;
    jmethodID sphinx4WrapperConstructor;
    jmethodID startMethod;
    jstring config;
    jobjectArray contextList;
    jstring currentContext;

    JavaVMInitArgs vm_args;
    JavaVMOption options[3];
    vm_args.nOptions = 3;

#if (CISST_OS == CISST_WINDOWS)
    const std::string pathDivider = ";";
#else
    const std::string pathDivider = ":";
#endif
    std::stringstream option;
    // first option
    option << "-Dmicrophone[selectMixer]=" << this->MicrophoneNumber;
    options[0].optionString = strdup(option.str().c_str());
    // second option
    option.str("");
    option << "-Djava.class.path=." << pathDivider
           << this->ModelFile << pathDivider
           << this->SphinxFile << pathDivider
           << this->SphinxWrapperDir;
    options[1].optionString = strdup(option.str().c_str());
    // third option
    option.str("");
    option << "-Djava.library.path=." << pathDivider
           << this->SphinxWrapperDir;
    options[2].optionString = strdup(option.str().c_str());
    // arguments for VM
    vm_args.options = options;
    vm_args.version = JNI_VERSION_1_6;
    vm_args.ignoreUnrecognized = JNI_FALSE;

    CMN_LOG_CLASS_INIT_DEBUG << "StartJava: starting Java with options: " << std::endl
                             << " -> " << options[0].optionString << std::endl
                             << " -> " << options[1].optionString << std::endl
                             << " -> " << options[2].optionString << std::endl;

    // create the Java VM
    result = JNI_CreateJavaVM(&(this->JavaData->VirtualMachine),
                              (void **) &(this->JavaData->Environment),
                              &vm_args);

    if (result < 0) {
        CMN_LOG_CLASS_INIT_ERROR << "StartJava: can't create Java VM" << std::endl;
        this->JavaData->VirtualMachine = 0;
        this->JavaData->Environment = 0;
        return false;
    }

    jclass sphinx4WrapperClass;
    sphinx4WrapperClass = this->JavaData->Environment->FindClass("sawCMUSphinx4");
    if (sphinx4WrapperClass == 0) {
        CMN_LOG_CLASS_INIT_ERROR << "StartJava: can't find Java class \"sawCMUSphinx4\"" << std::endl;
        return false;
    }

    sphinx4WrapperConstructor = this->JavaData->Environment->GetMethodID(sphinx4WrapperClass, "<init>", "()V");
    this->JavaData->CMUSphinx4Wrapper = this->JavaData->Environment->NewObject(sphinx4WrapperClass, sphinx4WrapperConstructor);
    // use command line utility "javap: to find exact signature:
    //  javap -s -p cscCMUSphinx4
    // (in build directory that has file cscCMUSphinx4.class

    // method used to change context/recognizer
    this->JavaData->SetCurrentContextMethod = this->JavaData->Environment->GetMethodID(sphinx4WrapperClass,
                                                                                       "SetCurrentContext",
                                                                                       "(Ljava/lang/String;)V");
    if (this->JavaData->SetCurrentContextMethod == 0) {
        CMN_LOG_CLASS_INIT_ERROR << "StartJava: can't find Java method \"sawCMUSphinx4.SetCurrentContextJavaMethod\"" << std::endl;
        return false;
    }

    // method used to configure/start sphinx4
    startMethod = this->JavaData->Environment->GetMethodID(sphinx4WrapperClass,
                                                           "Start",
                                                           "(JLjava/lang/String;[Ljava/lang/String;Ljava/lang/String;)V");
    if (startMethod == 0) {
        CMN_LOG_CLASS_INIT_ERROR << "StartJava: can't find Java method \"sawCMUSphinx4.Start\"" << std::endl;
        return false;
    }

    // send config file name
    config = this->JavaData->Environment->NewStringUTF(ConfigName.c_str());
    // send list on contexts
    contextList = this->JavaData->Environment->NewObjectArray(Contexts.size(),
                                                              this->JavaData->Environment->FindClass("java/lang/String"), 0);
    ContextMap::iterator iter;
    int i = 0;
    for (iter = Contexts.begin(); iter != Contexts.end(); iter++) {
        jstring context = this->JavaData->Environment->NewStringUTF(iter->first.c_str());
        this->JavaData->Environment->SetObjectArrayElement(contextList, i, context);
        i++;
    }
    // send starting context
    CMN_ASSERT(this->CurrentContext);
    currentContext = this->JavaData->Environment->NewStringUTF(CurrentContext->GetName().c_str());

    jlong thisPointer = (jlong)this;
    this->JavaData->Environment->CallVoidMethod(this->JavaData->CMUSphinx4Wrapper, startMethod,
                                                thisPointer, config, contextList, currentContext);
    this->JavaData->Environment->ReleaseStringUTFChars(config,
                                                       this->JavaData->Environment->GetStringUTFChars(config, JNI_FALSE));
    return true;
}


void mtsCMUSphinx4::Configure(void)
{
    // generate grammar files for each context
    // contextNameMap is a map from context name to grammar file name
    std::list<std::string> contextStrings;
    ContextMap::iterator cIter;
    for (cIter = Contexts.begin(); cIter != Contexts.end(); cIter++) {
        std::string contextName = cIter->first;
        Context * context = cIter->second;

        // generate grammar file
        std::ofstream fileStream((contextName + std::string(".gram")).c_str());
        fileStream <<
            "#JSGF V1.0;\n\n"
            "/**\n"
            " * Autogenerated JSGF Grammar for context " << contextName << "; do not edit\n"
            " */\n\n"
            "grammar " << contextName << ";\n\n";
        std::vector<std::string>::iterator wIter;
        std::vector<std::string> wordList = context->GetVocabulary();
        for (wIter = wordList.begin(); wIter != wordList.end(); wIter++) {
            std::string word = *wIter;
            // JSFG grammar cannot handle spaces in tokens
            std::string wordNoWhiteSpace = word;
            if (!ReplaceAll(wordNoWhiteSpace, " ", "")) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: error occurred while replacing spaces" << std::endl;
                return;
            }
            if (!ReplaceAll(wordNoWhiteSpace, "\t", "")) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: error occurred while replacing tabs" << std::endl;
                return;
            }
            fileStream << "public <" << wordNoWhiteSpace << "> = " << word << ";\n";
        }
        fileStream.close();

        std::stringstream contextStream;
        contextStream <<
            "    <component name=\"" << contextName << "Recognizer\"\n"
            "        type=\"edu.cmu.sphinx.recognizer.Recognizer\">\n"
            "        <property name=\"decoder\" value=\"" << contextName << "Decoder\"/>\n"
            "    </component>\n\n"

            "    <component name=\"" << contextName << "Decoder\"\n"
            "        type=\"edu.cmu.sphinx.decoder.Decoder\">\n"
            "        <property name=\"searchManager\" value=\"" << contextName << "SearchManager\"/>\n"
            "    </component>\n\n"

            "    <component name=\"" << contextName << "SearchManager\"\n"
            "        type=\"edu.cmu.sphinx.decoder.search.SimpleBreadthFirstSearchManager\">\n"
            "        <property name=\"logMath\" value=\"logMath\"/>\n"
            "        <property name=\"linguist\" value=\"" << contextName << "Linguist\"/>\n"
            "        <property name=\"pruner\" value=\"trivialPruner\"/>\n"
            "        <property name=\"scorer\" value=\"threadedScorer\"/>\n"
            "        <property name=\"activeListFactory\" value=\"activeList\"/>\n"
            "    </component>\n\n"

            "    <component name=\"" << contextName << "Linguist\"\n"
            "        type=\"edu.cmu.sphinx.linguist.flat.FlatLinguist\">\n"
            "        <property name=\"logMath\" value=\"logMath\"/>\n"
            "        <property name=\"grammar\" value=\"" << contextName << "Grammar\"/>\n"
            "        <property name=\"acousticModel\" value=\"wsj\"/>\n"
            "        <property name=\"wordInsertionProbability\"\n"
            "            value=\"${wordInsertionProbability}\"/>\n"
            "        <property name=\"languageWeight\" value=\"${languageWeight}\"/>\n"
            "        <property name=\"unitManager\" value=\"unitManager\"/>\n";
        if (context->FilteringEnabled()) {
            contextStream <<
            "        <property name=\"addOutOfGrammarBranch\" value=\"true\"/>\n"
            "        <property name=\"outOfGrammarProbability\" value=\"1E-20\"/>\n"
            "        <property name=\"phoneInsertionProbability\" value=\"1E-10\"/>\n"
            "        <property name=\"phoneLoopAcousticModel\" value=\"wsj\"/>\n";
        }
        contextStream <<
            "    </component>\n\n"

            "    <component name=\"" << contextName << "Grammar\" type=\"edu.cmu.sphinx.jsgf.JSGFGrammar\">\n"
            "        <property name=\"dictionary\" value=\"dictionary\"/>\n"
            "        <property name=\"grammarLocation\"\n"
            "             value=\"file:.\"/>\n"
            "        <property name=\"grammarName\" value=\"" << contextName << "\"/>\n"
            "        <property name=\"logMath\" value=\"logMath\"/>\n"
            "    </component>\n\n";
        contextStrings.push_back(contextStream.str());
    }

    std::list<std::string>::iterator iter;
    std::stringstream contextStream;
    for (iter = contextStrings.begin(); iter != contextStrings.end(); iter++) {
        contextStream << *iter;
    }
    std::string contextString = contextStream.str();

    // generate config file for this contextmanager containing
    // each context
    std::ifstream templateFile(TemplateFile.c_str());
    templateFile.seekg(0, std::ios::end);
    int length = templateFile.tellg();
    char * buffer = new char[length + 1];
    templateFile.seekg(0, std::ios::beg);
    templateFile.read(buffer, length);
    templateFile.close();
    buffer[length] = 0;
    std::string templateString = std::string(buffer);
    delete[] buffer;
    try {
        templateString.replace(templateString.find("    <!-- CONTEXT-SPECIFIC SECTION, TO BE REPLACED -->\n"),
                               std::string("    <!-- CONTEXT-SPECIFIC SECTION, TO BE REPLACED -->\n").length(),
                               contextString);
    } catch (std::exception & except) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: exception caught in replace from template \"" << TemplateFile << "\"" << std::endl;
        return;
    }
    if (!ReplaceAll(templateString, "___VAR_MODEL_TOP_DIR___", ModelTopDir)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: error occurred while replacing ___VAR_MODEL_TOP_DIR___" << std::endl;
        return;
    }

    std::ofstream fileStream(ConfigName.c_str());
    fileStream << templateString << std::endl << "<!-- end of file after configuration -->" << std::endl;
    fileStream.close();
}


void mtsCMUSphinx4::Startup(void)
{
    // make sure we have at least one context and current context is set
    if (this->Contexts.size() == 0) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: no context defined, you need to add at least one context using \"AddContext\"" << std::endl;
        return;
    }
    if (this->CurrentContext == 0) {
        CurrentContext = (this->Contexts.begin())->second;
        CMN_LOG_CLASS_INIT_WARNING << "Startup: no default context defined by user (using \"SetCurrentContext\"), will use \""
                                   << CurrentContext->GetName() << "\"" << std::endl;
    }
}


void mtsCMUSphinx4::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();
    if (this->JavaData->Environment == 0) {
        this->StartJava();
        this->ContextChangedTrigger(mtsStdString(this->CurrentContext->GetName()));
    }
    else {
        if (NewWordRecognized) {
            HandleWord(LastWordRecognized);
            NewWordRecognized = false;
        }
        // Wait for more commands/events
        osaSleep(15 * cmn_ms);
    }
}

void mtsCMUSphinx4::HandleWord(const mtsStdString & word) {
    // emit event as word has been recognized
    this->WordRecognizedTrigger->Execute(word, MTS_NOT_BLOCKING);
    // find word actions in context
    CMN_ASSERT(this->CurrentContext);
    this->CurrentContext->PerformActionsForWord(std::string(word));
}


void mtsCMUSphinx4::WordRecognizedCallback(const std::string & stdWord)
{
    if (stdWord != "" && stdWord != "<unk>") {
        NewWordRecognized = true;
        mtsStdString word(stdWord);
        // save last recognized word for state table
        this->LastWordRecognized = word;
    } else {
        if (stdWord == "<unk>")
            CMN_LOG_CLASS_INIT_DEBUG << "WordRecognizedCallback: word filtered out" << std::endl;
        else
            this->NoWordRecognizedTrigger->Execute(MTS_NOT_BLOCKING);
    }
}


void mtsCMUSphinx4::Cleanup(void)
{
    // todo, killing the java machine is a bit of a nightmare
    this->JavaData->VirtualMachine->DestroyJavaVM();
}
