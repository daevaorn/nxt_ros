#ifndef NXT_COLOR_DISPLAY_H
#define NXT_COLOR_DISPLAY_H

#include <rviz/message_filter_display.h>
#include <rviz/helpers/color.h>
#include <rviz/properties/float_property.h>

#include <nxt_msgs/Color.h>

#include <boost/shared_ptr.hpp>


namespace nxt_rviz_plugin
{

class NXTColorVisual;

/**
 * \class NXTColorDisplay
 * \brief Displays a nxt_msgs::Color message
 */
class NXTColorDisplay : public rviz::MessageFilterDisplay<nxt_msgs::Color>
{
Q_OBJECT
public:
  NXTColorDisplay();
  virtual ~NXTColorDisplay() {}

protected Q_SLOTS:
  void updateAlpha();
  void updateDisplayLength();

private:
  void processMessage(const nxt_msgs::Color::ConstPtr& msg);

  boost::shared_ptr<NXTColorVisual> visual_;

  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* display_length_property_;
};

} // namespace nxt_rviz_plugin

#endif /* NXT_COLOR_DISPLAY_H */

