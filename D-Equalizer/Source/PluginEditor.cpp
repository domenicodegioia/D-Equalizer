/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//====RESPONSE=CURVE=COMPONENT==================================================

ResponseCurveComponent::ResponseCurveComponent(EQAudioProcessor& p) :
audioProcessor(p),

leftPathProducer(audioProcessor.leftChannelFifo),
rightPathProducer(audioProcessor.rightChannelFifo)

{
    const auto& params = audioProcessor.getParameters();
    for (auto param : params) {
        param -> addListener(this);
    }
    
    updateChain();
    
    startTimerHz(60);
}

ResponseCurveComponent::~ResponseCurveComponent()
{
    const auto& params = audioProcessor.getParameters();
    for (auto param : params) {
        param -> removeListener(this);
    }
}

void ResponseCurveComponent::parameterValueChanged(int parameterIndex, float newValue)
{
    parametersChanged.set(true);
}

void PathProducer::process(juce::Rectangle<float> fftBounds, double sampleRate)
{
    juce::AudioBuffer<float> tempIncomingBuffer;
    
    while (leftChannelFifo->getNumCompleteBuffersAvailable() > 0) {
        if (leftChannelFifo->getAudioBuffer(tempIncomingBuffer)) {
            auto size = tempIncomingBuffer.getNumSamples();
            
            juce::FloatVectorOperations::copy(monoBuffer.getWritePointer(0, 0),
                                              monoBuffer.getReadPointer(0, size),
                                              monoBuffer.getNumSamples() - size);
            juce::FloatVectorOperations::copy(monoBuffer.getWritePointer(0, monoBuffer.getNumSamples() - size),
                                              tempIncomingBuffer.getReadPointer(0, 0),
                                              size);
            
            leftChannelFFTDataGenerator.produceFFTDataForRendering(monoBuffer, -48.0f);
        }
    }
    
    // if there are FFT data buffers to pull
        // if we can bull a buffer
            // generate a path
    
    const auto fftSize = leftChannelFFTDataGenerator.getFFTSize();
    const auto binWidth = sampleRate / (double) fftSize;
    
    while (leftChannelFFTDataGenerator.getNumAvailableFFTDataBlocks() > 0) {
        std::vector<float> fftData;
        if (leftChannelFFTDataGenerator.getFFTData(fftData)) {
            pathProducer.generatePath(fftData, fftBounds, fftSize, binWidth, -48.0f);
        }
    }
    
    // while there are paths that can be pull
        // pull as many as we can
            // display the most recent path
    
    while (pathProducer.getNumPathsAvailable()) {
        pathProducer.getPath(leftChannelFFTPath);
    }
}

void ResponseCurveComponent::timerCallback()
{
    auto fftBounds = getAnalysisArea().toFloat();
    auto sampleRate = audioProcessor.getSampleRate();
    
    leftPathProducer.process(fftBounds, sampleRate);
    rightPathProducer.process(fftBounds, sampleRate);
    
    if (parametersChanged.compareAndSetBool(false, true)) {
        updateChain();
    }
    
    repaint();
}

void ResponseCurveComponent::paint(juce::Graphics& g)
{
    g.fillAll(juce::Colour(14u, 14u, 14u));

    g.drawImage(background, getLocalBounds().toFloat());
    
    auto responseArea = getAnalysisArea();
    
    auto w = responseArea.getWidth();
    
    auto& lowcut = monoChain.get<ChainPositions::LowCut>();
    auto& peak = monoChain.get<ChainPositions::Peak>();
    auto& highcut = monoChain.get<ChainPositions::HighCut>();
    
    auto sampleRate = audioProcessor.getSampleRate();
    
    std::vector<double> mags;
    mags.resize(w);
    
    for (int i = 0; i < w; ++i) {
        double mag = 1.0f;
        auto freq = juce::mapToLog10(double (i) / double (w), 20.0, 20000.0);
        
        if (!monoChain.isBypassed<ChainPositions::Peak>())
            mag *= peak.coefficients -> getMagnitudeForFrequency(freq, sampleRate);
        
        if (!monoChain.isBypassed<ChainPositions::LowCut>())
        {
            if (!lowcut.isBypassed<0>())
                mag *= lowcut.get<0>().coefficients -> getMagnitudeForFrequency(freq, sampleRate);
            if (!lowcut.isBypassed<1>())
                mag *= lowcut.get<1>().coefficients -> getMagnitudeForFrequency(freq, sampleRate);
            if (!lowcut.isBypassed<2>())
                mag *= lowcut.get<2>().coefficients -> getMagnitudeForFrequency(freq, sampleRate);
            if (!lowcut.isBypassed<3>())
                mag *= lowcut.get<3>().coefficients -> getMagnitudeForFrequency(freq, sampleRate);
        }
        
        if (!monoChain.isBypassed<ChainPositions::HighCut>())
        {
            if (!highcut.isBypassed<0>())
                mag *= highcut.get<0>().coefficients -> getMagnitudeForFrequency(freq, sampleRate);
            if (!highcut.isBypassed<1>())
                mag *= highcut.get<1>().coefficients -> getMagnitudeForFrequency(freq, sampleRate);
            if (!highcut.isBypassed<2>())
                mag *= highcut.get<2>().coefficients -> getMagnitudeForFrequency(freq, sampleRate);
            if (!highcut.isBypassed<3>())
                mag *= highcut.get<3>().coefficients -> getMagnitudeForFrequency(freq, sampleRate);
        }
        
        mags[i] = juce::Decibels::gainToDecibels(mag);
    }
    
    // map function
    juce::Path responseCurve;
    const double outputMin = getAnalysisArea().getBottom();
    const double outputMax = getAnalysisArea().getY();
    
    auto map = [outputMin, outputMax](double input)
    {
        return juce::jmap(input, -24.0, 24.0, outputMin, outputMax);
    };
    
    responseCurve.startNewSubPath(responseArea.getX(), map(mags.front()));
    for (size_t i = 1; i < mags.size(); ++i) {
        responseCurve.lineTo(responseArea.getX() + i, map(mags[i]));
    }
    
    // SPECTRUM ANALYZER
    
    auto leftChannelFFTPath = leftPathProducer.getPath();
    leftChannelFFTPath.applyTransform(juce::AffineTransform().translation(responseArea.getX(), responseArea.getY()-11));
    g.setColour(juce::Colours::white);
    g.strokePath(leftChannelFFTPath, juce::PathStrokeType(1.0f));

    auto rightChannelFFTPath = rightPathProducer.getPath();
    rightChannelFFTPath.applyTransform(juce::AffineTransform().translation(responseArea.getX(), responseArea.getY()-11));
    g.setColour(juce::Colours::dimgrey);
    g.strokePath(rightChannelFFTPath, juce::PathStrokeType(1.0f));
    
    // draw border ResponseCurveComponen
    g.setColour(juce::Colours::white);
    g.drawRect(getAnalysisArea().toFloat());
    
    // draw responseCurve
    g.setColour(juce::Colours::orange);
    g.strokePath(responseCurve, juce::PathStrokeType(2.0f));
    
    //debug
    g.setColour(juce::Colour(14u, 14u, 14u));
    g.fillRect(getAnalysisArea().getWidth()+30, getAnalysisArea().getY(), getWidth()-(getAnalysisArea().getWidth()+30), getHeight()-getAnalysisArea().getY());
    g.fillRect(0, 261, getWidth(), 100);
}

void ResponseCurveComponent::resized()
{
    background = juce::Image(juce::Image::PixelFormat::RGB, getWidth(), getHeight(), true);
    
    juce::Graphics g(background);
    
    // FREQUENCY LINES
    
    juce::Array<float> freqs
    {
        20, 30, 40, 50, 100,
        200, 300, 400, 500, 1000,
        2000, 3000, 4000, 5000, 10000,
        20000
    };
    
    auto renderArea = getAnalysisArea();
    auto left = renderArea.getX();
    auto right = renderArea.getRight();
    auto top = renderArea.getY();
    auto bottom = renderArea.getBottom();
    auto width = renderArea.getWidth();
    
    juce::Array<float> xs;
    
    for (auto f : freqs) {
        auto normX = juce::mapFromLog10(f, 20.0f, 20000.0f);
        xs.add(left + width * normX);
    }
    
    g.setColour(juce::Colours::dimgrey);
    
    for (auto x : xs) {
        g.drawVerticalLine(x, top, bottom);
    }
    
    // GAIN LINES
    
    juce::Array<float> gain
    {
        -18, -12, -6, 0, +6, +12, +18,
    };
    
    for (auto gDb : gain) {
        auto y = juce::jmap(gDb, -24.0f, 24.0f, float(bottom), float(top));
        g.drawHorizontalLine(y, left, right);
    }
        
    // FREQUENCY LABELS
    
    g.setColour(juce::Colours::lightgrey);
    const int fontHeight = 10;
    g.setFont(fontHeight);
    
    for (int i = 0; i < freqs.size(); ++i) {
        auto f = freqs[i];
        auto x = xs[i];
        
        bool addK = false;
        
        //Composing string
        juce::String str;
        if (f > 999.0f) {
            addK = true;
            f /= 1000.0f;
        }
        str << f;
        if (addK) {
            str << "k";
        }
        str << "Hz";
        
        //Draw string
        auto textWidth = g.getCurrentFont().getStringWidth(str);
        
        juce::Rectangle<int> r;
        r.setSize(textWidth, fontHeight);
        r.setCentre(x, 0);
        r.setY(5);
        
        g.drawFittedText(str, r, juce::Justification::centred, 1);
    }
    
    // GAIN LABELS
    
    for (auto gDb : gain) {
        //Composing string
        auto y = juce::jmap(gDb, -24.0f, 24.0f, float(bottom), float(top));
        
        juce::String str;
        if (gDb > 0) {
            str << "+";
        }
        str << gDb;
        
        //Draw string
        auto textWidth = g.getCurrentFont().getStringWidth(str);
        
        juce::Rectangle<int> r;
        r.setSize(textWidth, fontHeight);
        r.setX(5);
        r.setCentre(r.getCentreX(), y);
        
        g.drawFittedText(str, r, juce::Justification::left, 1);
    }
}

juce::Rectangle<int> ResponseCurveComponent::getAnalysisArea()
{
    auto bounds = getLocalBounds();
    bounds.removeFromTop(24);
    bounds.removeFromBottom(14);
    bounds.removeFromLeft(30);
    bounds.removeFromRight(30);
    return bounds;
}

void ResponseCurveComponent::updateChain()
{
    auto chainSettings = getChainSettings(audioProcessor.apvts);
    
    monoChain.setBypassed<ChainPositions::LowCut>(chainSettings.lowCutBypassed);
    monoChain.setBypassed<ChainPositions::Peak>(chainSettings.peakBypassed);
    monoChain.setBypassed<ChainPositions::HighCut>(chainSettings.highCutBypassed);
    
    auto peakCoefficients = makePeakFilter(chainSettings, audioProcessor.getSampleRate());
    updateCoefficients(monoChain.get<ChainPositions::Peak>().coefficients, peakCoefficients);
    
    auto lowCutCoefficients = makeLowCutFilter(chainSettings, audioProcessor.getSampleRate());
    auto highCutCoefficients = makeHighCutFilter(chainSettings, audioProcessor.getSampleRate());
    
    updateCutFilter(monoChain.get<ChainPositions::LowCut>(), lowCutCoefficients, chainSettings.lowCutSlope);
    updateCutFilter(monoChain.get<ChainPositions::HighCut>(), highCutCoefficients, chainSettings.highCutSlope);
}

//==============================================================================

EQAudioProcessorEditor::EQAudioProcessorEditor (EQAudioProcessor& p)
    : AudioProcessorEditor (&p), audioProcessor (p),

peakFreqSliderAttachment(audioProcessor.apvts, "Peak Freq", peakFreqSlider),
peakGainSliderAttachment(audioProcessor.apvts, "Peak Gain", peakGainSlider),
peakQualitySliderAttachment(audioProcessor.apvts, "Peak Quality", peakQualitySlider),
lowCutFreqSliderAttachment(audioProcessor.apvts, "LowCut Freq", lowCutFreqSlider),
highCutFreqSliderAttachment(audioProcessor.apvts, "HighCut Freq", highCutFreqSlider),
lowCutSlopeSliderAttachment(audioProcessor.apvts, "LowCut Slope", lowCutSlopeSlider),
highCutSlopeSliderAttachment(audioProcessor.apvts, "HighCut Slope", highCutSlopeSlider),

lowcutBypassButtonAttachment(audioProcessor.apvts, "LowCut Bypassed", lowCutBypassButton),
peakBypassButtonAttachment(audioProcessor.apvts, "Peak Bypassed", peakBypassButton),
highcutBypassButtonAttachment(audioProcessor.apvts, "HighCut Bypassed", highCutBypassButton), 

responseCurveComponent(audioProcessor)

{
    for (auto* comp : getComps()) {
        addAndMakeVisible(comp);
    }
    
    //PEAK FILTER
    
    peakFreqSlider.setLookAndFeel(&lnf);
    peakFreqSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryVerticalDrag);
    peakFreqSlider.setTextValueSuffix(" Hz");
    peakFreqSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 100, 18);
    
    peakFreqLabel.setText("Frequency", juce::dontSendNotification);
    peakFreqLabel.setColour(juce::Label::textColourId, juce::Colours::orange);
    peakFreqLabel.setJustificationType(juce::Justification::centred);
    peakFreqLabel.attachToComponent(&peakFreqSlider, true);
    
    peakGainSlider.setLookAndFeel(&lnf);
    peakGainSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryVerticalDrag);
    peakGainSlider.setTextValueSuffix(" dB");
    peakGainSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 100, 18);
    
    peakGainLabel.setText("Gain", juce::dontSendNotification);
    peakGainLabel.setColour(juce::Label::textColourId, juce::Colours::orange);
    peakGainLabel.setJustificationType(juce::Justification::centred);
    peakGainLabel.attachToComponent(&peakGainSlider, true);
    
    peakQualitySlider.setLookAndFeel(&lnf);
    peakQualitySlider.setSliderStyle(juce::Slider::SliderStyle::RotaryVerticalDrag);
    peakQualitySlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 100, 18);
    
    peakQualityLabel.setText("Quality", juce::dontSendNotification);
    peakQualityLabel.setColour(juce::Label::textColourId, juce::Colours::orange);
    peakQualityLabel.setJustificationType(juce::Justification::centred);
    peakQualityLabel.attachToComponent(&peakQualitySlider, true);
    
    peakBypassLabel.setText("Bypass", juce::dontSendNotification);
    peakBypassLabel.setColour(juce::Label::textColourId, juce::Colours::orange);
    peakBypassLabel.setJustificationType(juce::Justification::centred);
    peakBypassLabel.attachToComponent(&peakBypassButton, true);
    
    //LOW CUT FILTER
    
    lowCutFreqSlider.setLookAndFeel(&lnf);
    lowCutFreqSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryVerticalDrag);
    lowCutFreqSlider.setTextValueSuffix(" Hz");
    lowCutFreqSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 100, 18);
    
    lowCutFreqLabel.setText("Frequency", juce::dontSendNotification);
    lowCutFreqLabel.setColour(juce::Label::textColourId, juce::Colours::orange);
    lowCutFreqLabel.setJustificationType(juce::Justification::centred);
    lowCutFreqLabel.attachToComponent(&lowCutFreqSlider, true);
    
    lowCutSlopeSlider.setLookAndFeel(&lnf);
    lowCutSlopeSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryVerticalDrag);
    lowCutSlopeSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 100, 18);
    
    lowCutSlopeLabel.setText("Slope", juce::dontSendNotification);
    lowCutSlopeLabel.setColour(juce::Label::textColourId, juce::Colours::orange);
    lowCutSlopeLabel.setJustificationType(juce::Justification::centred);
    lowCutSlopeLabel.attachToComponent(&lowCutSlopeSlider, true);
    
    lowCutBypassLabel.setText("Bypass", juce::dontSendNotification);
    lowCutBypassLabel.setColour(juce::Label::textColourId, juce::Colours::orange);
    lowCutBypassLabel.setJustificationType(juce::Justification::centred);
    lowCutBypassLabel.attachToComponent(&lowCutBypassButton, true);
    
    //HIGH CUT FILTER
    
    highCutFreqSlider.setLookAndFeel(&lnf);
    highCutFreqSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryVerticalDrag);
    highCutFreqSlider.setTextValueSuffix(" Hz");
    highCutFreqSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 100, 18);
    
    highCutFreqLabel.setText("Frequency", juce::dontSendNotification);
    highCutFreqLabel.setColour(juce::Label::textColourId, juce::Colours::orange);
    highCutFreqLabel.setJustificationType(juce::Justification::centred);
    highCutFreqLabel.attachToComponent(&highCutFreqSlider, true);
    
    highCutSlopeSlider.setLookAndFeel(&lnf);
    highCutSlopeSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryVerticalDrag);
    highCutSlopeSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 100, 18);
    
    highCutSlopeLabel.setText("Slope", juce::dontSendNotification);
    highCutSlopeLabel.setColour(juce::Label::textColourId, juce::Colours::orange);
    highCutSlopeLabel.setJustificationType(juce::Justification::centred);
    highCutSlopeLabel.attachToComponent(&highCutSlopeSlider, true);
    
    highCutBypassLabel.setText("Bypass", juce::dontSendNotification);
    highCutBypassLabel.setColour(juce::Label::textColourId, juce::Colours::orange);
    highCutBypassLabel.setJustificationType(juce::Justification::centred);
    highCutBypassLabel.attachToComponent(&highCutBypassButton, true);
    
    setSize (940, 620);
    
    setResizable(false, false);
}

EQAudioProcessorEditor::~EQAudioProcessorEditor()
{
}

//==============================================================================
void EQAudioProcessorEditor::paint (juce::Graphics& g)
{
    g.fillAll(juce::Colour(14u, 14u, 14u));
    
    g.setColour(juce::Colours::white);
    g.setFont(28);
    
    juce::Rectangle<int> lowcutText = lowCutSlopeSlider.getBounds();
    lowcutText.setY(lowcutText.getY()+50);
    lowcutText.setX(lowcutText.getX()-60);
    g.drawFittedText("LOWCUT", lowcutText, juce::Justification::centredBottom, 1);
    
    juce::Rectangle<int> peakText = peakQualitySlider.getBounds();
    peakText.setY(peakText.getY()+50);
    peakText.setX(peakText.getX()-65);
    g.drawFittedText("PEAK", peakText, juce::Justification::centredBottom, 1);
    
    juce::Rectangle<int> highcutText = highCutSlopeSlider.getBounds();
    highcutText.setY(highcutText.getY()+50);
    highcutText.setX(highcutText.getX()-55);
    g.drawFittedText("HIGHCUT", highcutText, juce::Justification::centredBottom, 1);
    
    g.drawVerticalLine(getLocalBounds().reduced(10).getWidth()*0.33, 290, getLocalBounds().getBottom() - 15);
    g.drawVerticalLine(getLocalBounds().reduced(10).getWidth()*0.66, 290, getLocalBounds().getBottom() - 15);
}

void EQAudioProcessorEditor::resized()
{
    auto bounds = getLocalBounds().reduced(10).removeFromTop(550);
    auto responseArea = bounds.removeFromTop(bounds.getHeight() * 0.5);
    
    responseCurveComponent.setBounds(responseArea);
    
    bounds.removeFromTop(5);
    
    auto lowCutArea = bounds.removeFromLeft(bounds.getWidth() * 0.33);
    auto highCutArea = bounds.removeFromRight(bounds.getWidth() * 0.5);
    
    auto lowcutBypassButtonPos = lowCutArea.removeFromTop(25);
    lowcutBypassButtonPos.setX(lowcutBypassButtonPos.getX() + 190 - 40);
    lowCutBypassButton.setBounds(lowcutBypassButtonPos);
    lowCutFreqSlider.setBounds(lowCutArea.removeFromTop(lowCutArea.getHeight() * 0.5).removeFromRight(200));
    lowCutSlopeSlider.setBounds(lowCutArea.removeFromRight(200));
    
    auto highcutBypassButtonPos = highCutArea.removeFromTop(25);
    highcutBypassButtonPos.setX(highcutBypassButtonPos.getX() + 196 - 40);
    highCutBypassButton.setBounds(highcutBypassButtonPos);
    highCutFreqSlider.setBounds(highCutArea.removeFromTop(highCutArea.getHeight() * 0.5).removeFromRight(200));
    highCutSlopeSlider.setBounds(highCutArea.removeFromRight(200));
    
    auto peakBypassButtonPos = bounds.removeFromTop(25);
    peakBypassButtonPos.setX(peakBypassButtonPos.getX() + 196 - 40);
    peakBypassButton.setBounds(peakBypassButtonPos);
    peakFreqSlider.setBounds(bounds.removeFromTop(bounds.getHeight() * 0.33).removeFromRight(200));
    peakGainSlider.setBounds(bounds.removeFromTop(bounds.getHeight() * 0.5).removeFromRight(200));
    peakQualitySlider.setBounds(bounds.removeFromRight(200));
}

// returns an array with component's references
std::vector<juce::Component*> EQAudioProcessorEditor::getComps()
{
    return
    {
        &peakFreqSlider,
        &peakGainSlider,
        &peakQualitySlider,
        &lowCutFreqSlider,
        &highCutFreqSlider,
        &lowCutSlopeSlider,
        &highCutSlopeSlider,
        
        &responseCurveComponent,
        
        &lowCutBypassButton,
        &peakBypassButton,
        &highCutBypassButton,
        
        &peakFreqLabel,
        &peakGainLabel,
        &peakQualityLabel,
        &lowCutFreqLabel,
        &highCutFreqLabel,
        &lowCutSlopeLabel,
        &highCutSlopeLabel
    };
}
