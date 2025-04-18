//#line 2 "/opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.h.template"
// *********************************************************
//
// File autogenerated for the stereo_dense_reconstruction package
// by the dynamic_reconfigure package.
// Please do not edit.
//
// ********************************************************/

#ifndef __stereo_dense_reconstruction__CAMTOROBOTCALIBPARAMSCONFIG_H__
#define __stereo_dense_reconstruction__CAMTOROBOTCALIBPARAMSCONFIG_H__

#if __cplusplus >= 201103L
#define DYNAMIC_RECONFIGURE_FINAL final
#else
#define DYNAMIC_RECONFIGURE_FINAL
#endif

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace stereo_dense_reconstruction
{
  class CamToRobotCalibParamsConfigStatics;

  class CamToRobotCalibParamsConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l,
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      virtual ~AbstractParamDescription() = default;

      virtual void clamp(CamToRobotCalibParamsConfig &config, const CamToRobotCalibParamsConfig &max, const CamToRobotCalibParamsConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const CamToRobotCalibParamsConfig &config1, const CamToRobotCalibParamsConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, CamToRobotCalibParamsConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const CamToRobotCalibParamsConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, CamToRobotCalibParamsConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const CamToRobotCalibParamsConfig &config) const = 0;
      virtual void getValue(const CamToRobotCalibParamsConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template <class T>
    class ParamDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string a_name, std::string a_type, uint32_t a_level,
          std::string a_description, std::string a_edit_method, T CamToRobotCalibParamsConfig::* a_f) :
        AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
        field(a_f)
      {}

      T CamToRobotCalibParamsConfig::* field;

      virtual void clamp(CamToRobotCalibParamsConfig &config, const CamToRobotCalibParamsConfig &max, const CamToRobotCalibParamsConfig &min) const override
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const CamToRobotCalibParamsConfig &config1, const CamToRobotCalibParamsConfig &config2) const override
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, CamToRobotCalibParamsConfig &config) const override
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const CamToRobotCalibParamsConfig &config) const override
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, CamToRobotCalibParamsConfig &config) const override
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const CamToRobotCalibParamsConfig &config) const override
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const CamToRobotCalibParamsConfig &config, boost::any &val) const override
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      virtual ~AbstractGroupDescription() = default;

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, CamToRobotCalibParamsConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template<class T, class PT>
    class GroupDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string a_name, std::string a_type, int a_parent, int a_id, bool a_s, T PT::* a_f) : AbstractGroupDescription(a_name, a_type, a_parent, a_id, a_s), field(a_f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, CamToRobotCalibParamsConfig &top) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const override
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T PT::* field;
      std::vector<CamToRobotCalibParamsConfig::AbstractGroupDescriptionConstPtr> groups;
    };

class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(CamToRobotCalibParamsConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("PHI_X"==(*_i)->name){PHI_X = boost::any_cast<double>(val);}
        if("PHI_Y"==(*_i)->name){PHI_Y = boost::any_cast<double>(val);}
        if("PHI_Z"==(*_i)->name){PHI_Z = boost::any_cast<double>(val);}
        if("TRANS_X"==(*_i)->name){TRANS_X = boost::any_cast<double>(val);}
        if("TRANS_Y"==(*_i)->name){TRANS_Y = boost::any_cast<double>(val);}
        if("TRANS_Z"==(*_i)->name){TRANS_Z = boost::any_cast<double>(val);}
      }
    }

    double PHI_X;
double PHI_Y;
double PHI_Z;
double TRANS_X;
double TRANS_Y;
double TRANS_Z;

    bool state;
    std::string name;

    
}groups;



//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double PHI_X;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double PHI_Y;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double PHI_Z;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double TRANS_X;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double TRANS_Y;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double TRANS_Z;
//#line 231 "/opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("CamToRobotCalibParamsConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }

    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }

    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const CamToRobotCalibParamsConfig &__max__ = __getMax__();
      const CamToRobotCalibParamsConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const CamToRobotCalibParamsConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }

    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const CamToRobotCalibParamsConfig &__getDefault__();
    static const CamToRobotCalibParamsConfig &__getMax__();
    static const CamToRobotCalibParamsConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

  private:
    static const CamToRobotCalibParamsConfigStatics *__get_statics__();
  };

  template <> // Max and min are ignored for strings.
  inline void CamToRobotCalibParamsConfig::ParamDescription<std::string>::clamp(CamToRobotCalibParamsConfig &config, const CamToRobotCalibParamsConfig &max, const CamToRobotCalibParamsConfig &min) const
  {
    (void) config;
    (void) min;
    (void) max;
    return;
  }

  class CamToRobotCalibParamsConfigStatics
  {
    friend class CamToRobotCalibParamsConfig;

    CamToRobotCalibParamsConfigStatics()
    {
CamToRobotCalibParamsConfig::GroupDescription<CamToRobotCalibParamsConfig::DEFAULT, CamToRobotCalibParamsConfig> Default("Default", "", 0, 0, true, &CamToRobotCalibParamsConfig::groups);
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.PHI_X = -6.28;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.PHI_X = 6.28;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.PHI_X = 1.3;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr(new CamToRobotCalibParamsConfig::ParamDescription<double>("PHI_X", "double", 0, "Euler angle X", "", &CamToRobotCalibParamsConfig::PHI_X)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr(new CamToRobotCalibParamsConfig::ParamDescription<double>("PHI_X", "double", 0, "Euler angle X", "", &CamToRobotCalibParamsConfig::PHI_X)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.PHI_Y = -6.28;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.PHI_Y = 6.28;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.PHI_Y = -3.14;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr(new CamToRobotCalibParamsConfig::ParamDescription<double>("PHI_Y", "double", 0, "Euler angle Y", "", &CamToRobotCalibParamsConfig::PHI_Y)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr(new CamToRobotCalibParamsConfig::ParamDescription<double>("PHI_Y", "double", 0, "Euler angle Y", "", &CamToRobotCalibParamsConfig::PHI_Y)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.PHI_Z = -6.28;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.PHI_Z = 6.28;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.PHI_Z = 1.57;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr(new CamToRobotCalibParamsConfig::ParamDescription<double>("PHI_Z", "double", 0, "Euler angle Z", "", &CamToRobotCalibParamsConfig::PHI_Z)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr(new CamToRobotCalibParamsConfig::ParamDescription<double>("PHI_Z", "double", 0, "Euler angle Z", "", &CamToRobotCalibParamsConfig::PHI_Z)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.TRANS_X = -100.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.TRANS_X = 100.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.TRANS_X = 0.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr(new CamToRobotCalibParamsConfig::ParamDescription<double>("TRANS_X", "double", 0, "Translation in X-axis", "", &CamToRobotCalibParamsConfig::TRANS_X)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr(new CamToRobotCalibParamsConfig::ParamDescription<double>("TRANS_X", "double", 0, "Translation in X-axis", "", &CamToRobotCalibParamsConfig::TRANS_X)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.TRANS_Y = -100.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.TRANS_Y = 100.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.TRANS_Y = 0.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr(new CamToRobotCalibParamsConfig::ParamDescription<double>("TRANS_Y", "double", 0, "Translation in Y-axis", "", &CamToRobotCalibParamsConfig::TRANS_Y)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr(new CamToRobotCalibParamsConfig::ParamDescription<double>("TRANS_Y", "double", 0, "Translation in Y-axis", "", &CamToRobotCalibParamsConfig::TRANS_Y)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.TRANS_Z = -100.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.TRANS_Z = 100.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.TRANS_Z = 0.28;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr(new CamToRobotCalibParamsConfig::ParamDescription<double>("TRANS_Z", "double", 0, "Translation in Z-axis", "", &CamToRobotCalibParamsConfig::TRANS_Z)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr(new CamToRobotCalibParamsConfig::ParamDescription<double>("TRANS_Z", "double", 0, "Translation in Z-axis", "", &CamToRobotCalibParamsConfig::TRANS_Z)));
//#line 245 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.convertParams();
//#line 245 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __group_descriptions__.push_back(CamToRobotCalibParamsConfig::AbstractGroupDescriptionConstPtr(new CamToRobotCalibParamsConfig::GroupDescription<CamToRobotCalibParamsConfig::DEFAULT, CamToRobotCalibParamsConfig>(Default)));
//#line 369 "/opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.h.template"

      for (std::vector<CamToRobotCalibParamsConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
    }
    std::vector<CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<CamToRobotCalibParamsConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    CamToRobotCalibParamsConfig __max__;
    CamToRobotCalibParamsConfig __min__;
    CamToRobotCalibParamsConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const CamToRobotCalibParamsConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static CamToRobotCalibParamsConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &CamToRobotCalibParamsConfig::__getDescriptionMessage__()
  {
    return __get_statics__()->__description_message__;
  }

  inline const CamToRobotCalibParamsConfig &CamToRobotCalibParamsConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }

  inline const CamToRobotCalibParamsConfig &CamToRobotCalibParamsConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }

  inline const CamToRobotCalibParamsConfig &CamToRobotCalibParamsConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }

  inline const std::vector<CamToRobotCalibParamsConfig::AbstractParamDescriptionConstPtr> &CamToRobotCalibParamsConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<CamToRobotCalibParamsConfig::AbstractGroupDescriptionConstPtr> &CamToRobotCalibParamsConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const CamToRobotCalibParamsConfigStatics *CamToRobotCalibParamsConfig::__get_statics__()
  {
    const static CamToRobotCalibParamsConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = CamToRobotCalibParamsConfigStatics::get_instance();

    return statics;
  }


}

#undef DYNAMIC_RECONFIGURE_FINAL

#endif // __CAMTOROBOTCALIBPARAMSRECONFIGURATOR_H__
