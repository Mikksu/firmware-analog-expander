/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <BitmapDatabase.hpp>

Screen1ViewBase::Screen1ViewBase()
{

    __background.setPosition(0, 0, 240, 135);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));

    scalableImage1.setBitmap(touchgfx::Bitmap(BITMAP_STOP_SIGN_ID));
    scalableImage1.setPosition(70, 13, 101, 108);
    scalableImage1.setScalingAlgorithm(touchgfx::ScalableImage::NEAREST_NEIGHBOR);

    add(__background);
    add(scalableImage1);
}

void Screen1ViewBase::setupScreen()
{

}