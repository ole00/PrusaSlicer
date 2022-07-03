#include "pwmx.hpp"
#include "GCode/ThumbnailData.hpp"
#include "SLA/RasterBase.hpp"
#include "libslic3r/SLAPrint.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/log/trivial.hpp>


#define TAG_INTRO "ANYCUBIC\0\0\0\0"
#define TAG_HEADER "HEADER\0\0\0\0\0\0"
#define TAG_PREVIEW "PREVIEW\0\0\0\0\0"
#define TAG_LAYERS "LAYERDEF\0\0\0\0"
#define TAG_EXTRA   "EXTRA\0\0\0\0\0\0\0"
#define TAG_MACHINE "MACHINE\0\0\0\0\0"

#define CFG_LIFT_DISTANCE "LIFT_DISTANCE"
#define CFG_LIFT_SPEED "LIFT_SPEED"
#define CFG_RETRACT_SPEED "RETRACT_SPEED"

#define CFG_EXTRA_LIFT_DISTANCE "EXTRA_LIFT_DISTANCE"
#define CFG_EXTRA_LIFT_SPEED "EXTRA_LIFT_SPEED"
#define CFG_EXTRA_RETRACT_SPEED "EXTRA_RETRACT_SPEED"


#define CFG_DELAY_BEFORE_EXPOSURE "DELAY_BEFORE_EXPOSURE"
#define CFG_BOTTOM_LIFT_SPEED "BOTTOM_LIFT_SPEED"
#define CFG_BOTTOM_LIFT_DISTANCE "BOTTOM_LIFT_DISTANCE"
#define CFG_EXPORT_VERSION "PRINTER_EXPORT_PWM_VER"
#define CFG_EXPORT_MACHINE_NAME "PRINTER_EXPORT_PWM_NAME"


#define PREV_W 224
#define PREV_H 168
#define PREV_DPI 42

#define LAYER_SIZE_ESTIMATE (32 * 1024)


namespace Slic3r {

static void pwx_get_pixel_span(const std::uint8_t* ptr, const std::uint8_t* end,
                               std::uint8_t& pixel, size_t& span_len)
{
    size_t max_len;

    span_len = 0;
    pixel = (*ptr) & 0xF0;
    // the maximum length of the span depends on the pixel color
    max_len = (pixel == 0 || pixel == 0xF0) ? 0xFFF : 0xF;
    while (ptr < end && span_len < max_len && ((*ptr) & 0xF0) == pixel) {
        span_len++;
        ptr++;
    }
}

struct PWXRasterEncoder
{
    sla::EncodedRaster operator()(const void *ptr,
                                  size_t      w,
                                  size_t      h,
                                  size_t      num_components)
    {
        std::vector<uint8_t> dst;
        size_t               span_len;
        std::uint8_t         pixel;
        auto                 size = w * h * num_components;
        dst.reserve(size);

        const std::uint8_t *src = reinterpret_cast<const std::uint8_t *>(ptr);
        const std::uint8_t *src_end = src + size;
        while (src < src_end) {
            pwx_get_pixel_span(src, src_end, pixel, span_len);
            src += span_len;
            // fully transparent of fully opaque pixel
            if (pixel == 0 || pixel == 0xF0) {
                pixel = pixel | (span_len >> 8);
                std::copy(&pixel, (&pixel) + 1, std::back_inserter(dst));
                pixel = span_len & 0xFF;
                std::copy(&pixel, (&pixel) + 1, std::back_inserter(dst));
            }
            // antialiased pixel
            else {
                pixel = pixel | span_len;
                std::copy(&pixel, (&pixel) + 1, std::back_inserter(dst));
            }
        }

        return sla::EncodedRaster(std::move(dst), "pwx");
    }
};

using ConfMap = std::map<std::string, std::string>;

typedef struct pwmx_format_intro
{
    char          tag[12];
    std::uint32_t version;  // value 1
    std::uint32_t area_num; // unknown - usually 4
    std::uint32_t header_data_offset;
    std::float_t  intro24; // unknown - usually 0
    std::uint32_t preview_data_offset;
    std::float_t  intro32; // unknown
    std::uint32_t layer_data_offset;
    std::float_t  intro40; // unknown
    std::uint32_t image_data_offset;
} pwmx_format_intro;

typedef struct pwmx_format_intro_v516
{
    char          tag[12];
    std::uint32_t version;  // value 1
    std::uint32_t area_num; // unknown - usually 4
    std::uint32_t header_data_offset;
    std::float_t  intro24; // unknown - usually 0
    std::uint32_t preview_data_offset;
    std::float_t  intro32; // preview unknown data
    std::uint32_t layer_data_offset;
    std::uint32_t extra_data_offset;
    std::uint32_t machine_data_offset;
    std::uint32_t image_data_offset;
} pwmx_format_intro_v516;

typedef struct pwmx_format_header
{
    char          tag[12];
    std::uint32_t payload_size;
    std::float_t  pixel_size_um;
    std::float_t  layer_height_mm;
    std::float_t  exposure_time_s;
    std::float_t  delay_before_exposure_s;
    std::float_t  bottom_exposure_time_s;
    std::float_t  bottom_layer_count;
    std::float_t  lift_distance_mm;
    std::float_t  lift_speed_mms;
    std::float_t  retract_speed_mms;
    std::float_t  volume_ml;
    std::uint32_t antialiasing;
    std::uint32_t res_x;
    std::uint32_t res_y;
    std::float_t  weight_g;
    std::float_t  price;
    std::uint32_t price_currency;
    std::uint32_t per_layer_override; // ? unknown meaning ?
    std::uint32_t print_time_s;
    std::uint32_t transition_layer_count;
    std::uint32_t unknown; // ? usually 0 ?

} pwmx_format_header;

typedef struct pwmx_format_extra
{
    char          tag[12];
    std::uint32_t extra0;
    std::uint32_t extra4;
    std::float_t  lift_distance1_mm;
    std::float_t  lift_speed1_mms;
    std::float_t  retract_speed1_mms;
    std::float_t  lift_distance2_mm;
    std::float_t  lift_speed2_mms;
    std::float_t  retract_speed2_mms;
    std::uint32_t extra32;
    std::float_t  lift_distance3_mm;
    std::float_t  lift_speed3_mms;
    std::float_t  retract_speed3_mms;
    std::float_t  lift_distance4_mm;
    std::float_t  lift_speed4_mms;
    std::float_t  retract_speed4_mms;
} pwmx_format_extra;

typedef struct pwmx_format_machine
{
    char          tag[12];
    std::uint32_t payload_size;
    char          name[96];
    char          image_format[24];
    std::float_t  volume_x;
    std::float_t  volume_y;
    std::float_t  volume_z;
    std::uint32_t version;
    std::uint32_t machine140;
} pwmx_format_machine;

typedef struct pwmx_format_preview
{
    char          tag[12];
    std::uint32_t payload_size;
    std::uint32_t preview_w;
    std::uint32_t preview_dpi;
    std::uint32_t preview_h;
    // raw image data in BGR565 format
     std::uint8_t pixels[PREV_W * PREV_H * 2];
} pwmx_format_preview;

typedef struct pwmx_format_preview_unknown_v516
{
    std::uint32_t unknown[7];
} pwmx_format_preview_unknown_v516;

typedef struct pwmx_format_layers_header
{
    char          tag[12];
    std::uint32_t payload_size;
    std::uint32_t layer_count;
} pwmx_format_layers_header;

typedef struct pwmx_format_layer
{
    std::uint32_t image_offset;
    std::uint32_t image_size;
    std::float_t  lift_distance_mm;
    std::float_t  lift_speed_mms;
    std::float_t  exposure_time_s;
    std::float_t  layer_height_mm;
    std::float_t  layer44; // unkown - usually 0
    std::float_t  layer48; // unkown - usually 0
} pwmx_format_layer;

typedef struct pwmx_format_misc
{
    std::float_t bottom_layer_height_mm;
    std::float_t bottom_lift_distance_mm;
    std::float_t bottom_lift_speed_mms;

} pwmx_format_misc;

class PwmxFormatConfigDef : public ConfigDef
{
public:
    PwmxFormatConfigDef()
    {
        add(CFG_LIFT_DISTANCE, coFloat);
        add(CFG_LIFT_SPEED, coFloat);
        add(CFG_RETRACT_SPEED, coFloat);
        add(CFG_DELAY_BEFORE_EXPOSURE, coFloat);
        add(CFG_BOTTOM_LIFT_DISTANCE, coFloat);
        add(CFG_BOTTOM_LIFT_SPEED, coFloat);
    }
};

class PwmxFormatDynamicConfig : public DynamicConfig
{
public:
    PwmxFormatDynamicConfig(){};
    const ConfigDef *def() const override { return &config_def; }

private:
    PwmxFormatConfigDef config_def;
};

namespace {

std::float_t get_cfg_value_f(const DynamicConfig &cfg,
                             const std::string   &key,
                             const std::float_t  &def = 0.f)
{
    if (cfg.has(key)) {
        if (auto opt = cfg.option(key))
            return opt->getFloat();
    }

    return def;
}

int get_cfg_value_i(const DynamicConfig &cfg,
                    const std::string   &key,
                    const int           &def = 0)
{
    if (cfg.has(key)) {
        if (auto opt = cfg.option(key))
            return opt->getInt();
    }

    return def;
}

std::string get_vec_value_s(const std::vector<std::string> &items,
                            const std::string   &key,
                            const std::string   &def = "")
{
    for (size_t i = 0; i < items.size(); i++) {
        int index = items[i].find(key);
        if (0 == index) {
            std::vector<std::string> tokens(2);
            boost::split(tokens, items[i], boost::is_any_of("\t ="), boost::token_compress_on);
            if (2 != tokens.size()) { // only a key/value pair is allowed
	            throw Slic3r::IOError("invalid config value of key: " + key);
            }
            std::string result = tokens[1];
            boost::replace_all(result, "__", " ");
            return result;
        }
    }
    return def;
}

int get_vec_value_i(const std::vector<std::string> &items,
                    const std::string   &key,
                    int   def = 0)
{
    std::string result = get_vec_value_s(items, key);
    if (result.empty()) {
        return def;
    }
    return std::stoi(result.c_str());
}

template<class T> void crop_value(T &val, T val_min, T val_max)
{
    if (val < val_min) {
        val = val_min;
    } else if (val > val_max) {
        val = val_max;
    }
}

void fill_preview(pwmx_format_preview &p,
                  pwmx_format_preview_unknown_v516 &pu,
                  pwmx_format_misc   &/*m*/,
                  const ThumbnailsList &thumbnails,
                  bool v516)
{

    p.preview_w    = PREV_W;
    p.preview_h    = PREV_H;
    p.preview_dpi  = PREV_DPI;
    //v1 calculates the payload size incorrectly
    p.payload_size = sizeof(p) - sizeof(p.tag) - sizeof(p.payload_size);
                     
    std::memset(p.pixels, 0 , sizeof(p.pixels));
    if (!thumbnails.empty()) {
        std::uint32_t dst_index;
        std::uint32_t i = 0;
        size_t len;
        size_t pixel_x = 0;
        auto t = thumbnails[0]; //use the first thumbnail
        len = t.pixels.size();
        //sanity check        
        if (len != PREV_W * PREV_H * 4)  {
            printf("incorrect thumbnail size. expected %ix%i\n", PREV_W, PREV_H);
            return;
        }
        // rearange pixels: they seem to be stored from bottom to top.
        dst_index = (PREV_W * (PREV_H - 1) * 2);
        while (i < len) {
            std::uint32_t pixel;
            std::uint32_t r = t.pixels[i++];
            std::uint32_t g = t.pixels[i++];
            std::uint32_t b = t.pixels[i++];
            i++; // Alpha
            // convert to BGRA565
            pixel = ((b >> 3) << 11) | ((g >>2) << 5) | (r >> 3);
            p.pixels[dst_index++] = pixel & 0xFF;
            p.pixels[dst_index++] = (pixel >> 8) & 0xFF;
            pixel_x++;
            if (pixel_x == PREV_W) {
                pixel_x = 0;
                dst_index -= (PREV_W * 4);
            }
        }
    }

    if (v516) {
        // payload size now fixed
        p.payload_size = sizeof(p);
        // fill unknown preview data with a knownn contents (text or bg palette perhaps?)
        pu.unknown[0] = 0;
        pu.unknown[1] = 0x10;
        pu.unknown[2] = 0xFFFFFFFF;
        pu.unknown[3] = 0xFFFFFFFF;
        pu.unknown[4] = 0xFFFFFFFF;
        pu.unknown[5] = 0xFFFFFFFF;
        pu.unknown[6] = 0;
    }
}

int get_format_version(const SLAPrint &print)
{
    auto        &cfg      = print.full_print_config();
    auto        print_opt = cfg.option("printer_notes");
    std::string pnotes    = print_opt? cfg.option("printer_notes")->serialize() : "";
    // create a vector of strings from the printer notes delimited by new line characters
    std::vector<std::string> pnotes_items;

    // sanitize the string config
    boost::replace_all(pnotes, "\\n", "\n");
    boost::replace_all(pnotes, "\\r", "\r");
    boost::split(pnotes_items, pnotes, boost::is_any_of("\n\r"), boost::token_compress_on);

    int result = get_vec_value_i(pnotes_items, CFG_EXPORT_VERSION, 1);
    return result;
}

void fill_header(pwmx_format_header &h,
                 pwmx_format_misc   &m,
                 const SLAPrint     &print,
                 std::uint32_t       layer_count)
{
    CNumericLocalesSetter locales_setter;

    std::float_t bottle_weight_g;
    std::float_t bottle_volume_ml;
    std::float_t bottle_cost;
    std::float_t material_density;
    auto        &cfg     = print.full_print_config();
    auto         mat_opt = cfg.option("material_notes");
    std::string  mnotes  = mat_opt? cfg.option("material_notes")->serialize() : "";
    // create a config parser from the material notes
    Slic3r::PwmxFormatDynamicConfig mat_cfg;
    SLAPrintStatistics              stats = print.print_statistics();

    // sanitize the string config
    boost::replace_all(mnotes, "\\n", "\n");
    boost::replace_all(mnotes, "\\r", "\r");
    mat_cfg.load_from_ini_string(mnotes,
                                 ForwardCompatibilitySubstitutionRule::Enable);

    h.layer_height_mm        = get_cfg_value_f(cfg, "layer_height");
    m.bottom_layer_height_mm = get_cfg_value_f(cfg, "initial_layer_height");
    h.exposure_time_s        = get_cfg_value_f(cfg, "exposure_time");
    h.bottom_exposure_time_s = get_cfg_value_f(cfg, "initial_exposure_time");
    h.bottom_layer_count =     get_cfg_value_i(cfg, "faded_layers");
    if (layer_count < h.bottom_layer_count) {
        h.bottom_layer_count = layer_count;
    }
    h.res_x     = get_cfg_value_i(cfg, "display_pixels_x");
    h.res_y     = get_cfg_value_i(cfg, "display_pixels_y");

    auto         dispo_opt = cfg.option("display_orientation");
    std::string  dispo  = dispo_opt? cfg.option("display_orientation")->serialize() : "l";
    if (dispo == "portrait") {
        std::swap(h.res_x, h.res_y);
    }

    bottle_weight_g = get_cfg_value_f(cfg, "bottle_weight") * 1000.0f;
    bottle_volume_ml = get_cfg_value_f(cfg, "bottle_volume");
    bottle_cost = get_cfg_value_f(cfg, "bottle_cost");
    material_density = bottle_weight_g / bottle_volume_ml;

    h.volume_ml = (stats.objects_used_material + stats.support_used_material) / 1000;
    h.weight_g           = h.volume_ml * material_density;
    h.price              = (h.volume_ml * bottle_cost) /  bottle_volume_ml;
    h.price_currency     = '$';
    h.antialiasing       = 1;
    h.per_layer_override = 0;

    // TODO - expose these variables to the UI rather than using material notes
    h.delay_before_exposure_s = get_cfg_value_f(mat_cfg, CFG_DELAY_BEFORE_EXPOSURE, 0.5f);
    crop_value(h.delay_before_exposure_s, 0.0f, 1000.0f);

    h.lift_distance_mm = get_cfg_value_f(mat_cfg, CFG_LIFT_DISTANCE, 8.0f);
    crop_value(h.lift_distance_mm, 0.0f, 100.0f);

    if (mat_cfg.has(CFG_BOTTOM_LIFT_DISTANCE)) {
        m.bottom_lift_distance_mm = get_cfg_value_f(mat_cfg,
                                                    CFG_BOTTOM_LIFT_DISTANCE,
                                                    8.0f);
        crop_value(h.lift_distance_mm, 0.0f, 100.0f);
    } else {
        m.bottom_lift_distance_mm = h.lift_distance_mm;
    }

    h.lift_speed_mms = get_cfg_value_f(mat_cfg, CFG_LIFT_SPEED, 2.0f);
    crop_value(m.bottom_lift_speed_mms, 0.1f, 20.0f);

    if (mat_cfg.has(CFG_BOTTOM_LIFT_SPEED)) {
        m.bottom_lift_speed_mms = get_cfg_value_f(mat_cfg, CFG_BOTTOM_LIFT_SPEED, 2.0f);
        crop_value(m.bottom_lift_speed_mms, 0.1f, 20.0f);
    } else {
        m.bottom_lift_speed_mms = h.lift_speed_mms;
    }

    h.retract_speed_mms = get_cfg_value_f(mat_cfg, CFG_RETRACT_SPEED, 3.0f);
    crop_value(h.lift_speed_mms, 0.1f, 20.0f);

    h.print_time_s = (h.bottom_layer_count * h.bottom_exposure_time_s) +
                     ((layer_count - h.bottom_layer_count) *
                      h.exposure_time_s) +
                     (layer_count * h.lift_distance_mm / h.retract_speed_mms) +
                     (layer_count * h.lift_distance_mm / h.lift_speed_mms) +
                     (layer_count * h.delay_before_exposure_s);


    h.payload_size  = sizeof(h) - sizeof(h.tag) - sizeof(h.payload_size);

    std::float_t display_w = get_cfg_value_f(cfg, "display_width", 100) * 1000;
    if (dispo == "portrait") {
        h.pixel_size_um = (int)((display_w / h.res_y) + 0.5f);
    } else {
        h.pixel_size_um = (int)((display_w / h.res_x) + 0.5f);
    }
}

void fill_extra(pwmx_format_extra  &e,
                const SLAPrint     &print)
{
    auto        &cfg     = print.full_print_config();
    auto         mat_opt = cfg.option("material_notes");
    std::string  mnotes  = mat_opt? cfg.option("material_notes")->serialize() : "";
    // create a config parser from the material notes
    Slic3r::PwmxFormatDynamicConfig mat_cfg;

    // sanitize the string config
    boost::replace_all(mnotes, "\\n", "\n");
    boost::replace_all(mnotes, "\\r", "\r");
    mat_cfg.load_from_ini_string(mnotes,
                                 ForwardCompatibilitySubstitutionRule::Enable);

    // unknown fields - the values from TEST.pwma are used
    e.extra0 = 24;
    e.extra4 = 2;
    e.extra32 = 2;

    // Currently it is unknown when (during printing) these values are applied
    // and which values (layer section or extra section) have higher priority.
    // These configurtion options can be set in material notes.

    e.lift_distance1_mm  = get_cfg_value_f(mat_cfg, CFG_EXTRA_LIFT_DISTANCE "1", 1.5f);;
    e.lift_speed1_mms    = get_cfg_value_f(mat_cfg, CFG_EXTRA_LIFT_SPEED "1", 2.0f);;
    e.retract_speed1_mms = get_cfg_value_f(mat_cfg, CFG_EXTRA_RETRACT_SPEED "1", 3.0f);;

    e.lift_distance2_mm  = get_cfg_value_f(mat_cfg, CFG_EXTRA_LIFT_DISTANCE "2", 4.5f);;
    e.lift_speed2_mms    = get_cfg_value_f(mat_cfg, CFG_EXTRA_LIFT_SPEED "2", 4.0f);;
    e.retract_speed2_mms = get_cfg_value_f(mat_cfg, CFG_EXTRA_RETRACT_SPEED "2", 6.0f);;

    e.lift_distance3_mm  = get_cfg_value_f(mat_cfg, CFG_EXTRA_LIFT_DISTANCE "3", 1.5f);;
    e.lift_speed3_mms    = get_cfg_value_f(mat_cfg, CFG_EXTRA_LIFT_SPEED "3", 2.0f);;
    e.retract_speed3_mms = get_cfg_value_f(mat_cfg, CFG_EXTRA_RETRACT_SPEED "3", 3.0f);;

    e.lift_distance4_mm  = get_cfg_value_f(mat_cfg, CFG_EXTRA_LIFT_DISTANCE "4", 4.0f);;
    e.lift_speed4_mms    = get_cfg_value_f(mat_cfg, CFG_EXTRA_LIFT_SPEED "4", 2.0f);;
    e.retract_speed4_mms = get_cfg_value_f(mat_cfg, CFG_EXTRA_RETRACT_SPEED "4", 3.0f);;

    // ensure sane values are set
    crop_value(e.lift_distance1_mm, 0.1f, 100.0f);
    crop_value(e.lift_distance2_mm, 0.1f, 100.0f);
    crop_value(e.lift_distance3_mm, 0.1f, 100.0f);
    crop_value(e.lift_distance4_mm, 0.1f, 100.0f);

    crop_value(e.lift_speed1_mms, 0.1f, 20.0f);
    crop_value(e.lift_speed2_mms, 0.1f, 20.0f);
    crop_value(e.lift_speed3_mms, 0.1f, 20.0f);
    crop_value(e.lift_speed4_mms, 0.1f, 20.0f);

    crop_value(e.retract_speed1_mms, 0.1f, 20.0f);
    crop_value(e.retract_speed2_mms, 0.1f, 20.0f);
    crop_value(e.retract_speed3_mms, 0.1f, 20.0f);
    crop_value(e.retract_speed4_mms, 0.1f, 20.0f);
}

void fill_machine(pwmx_format_machine &m,
                  const SLAPrint      &print,
                  int                 version)
{
    auto        &cfg     = print.full_print_config();
    auto         mat_opt = cfg.option("material_notes");
    std::string  mnotes  = mat_opt? cfg.option("material_notes")->serialize() : "";
    // create a config parser from the material notes
    Slic3r::PwmxFormatDynamicConfig mat_cfg;
    auto        print_opt = cfg.option("printer_notes");
    std::string pnotes    = print_opt? cfg.option("printer_notes")->serialize() : "";
    // create a vector of strings from the printer notes
    std::vector<std::string> pnotes_items;

    // sanitize the  material notes
    boost::replace_all(mnotes, "\\n", "\n");
    boost::replace_all(mnotes, "\\r", "\r");
    mat_cfg.load_from_ini_string(mnotes,
                                 ForwardCompatibilitySubstitutionRule::Enable);

    // sanitize the printer notes
    boost::replace_all(pnotes, "\\n", "\n");
    boost::replace_all(pnotes, "\\r", "\r");
    boost::split(pnotes_items, pnotes, boost::is_any_of("\n\r"), boost::token_compress_on);

    std::string name = get_vec_value_s(pnotes_items, CFG_EXPORT_MACHINE_NAME, "Photon Mono");
    std::strncpy((char*) m.name, name.c_str(), sizeof(m.name));
    std::strncpy((char*) m.image_format, "pw0Img", sizeof(m.image_format));

    m.volume_x = get_cfg_value_f(cfg, "display_width");
    m.volume_y = get_cfg_value_f(cfg, "display_height");
    m.volume_z = get_cfg_value_f(cfg, "max_print_height", 160);
    m.version  = version;
    m.machine140 = 0x634701; // unknown purpose (found in  TEST.pwma  - Photon Mono 4K)
    m.payload_size = sizeof(m);

    auto dispo_opt = cfg.option("display_orientation");
    std::string dispo = dispo_opt? cfg.option("display_orientation")->serialize() : "l";
    if (dispo == "portrait") {
        std::swap(m.volume_x, m.volume_y);
    }
}

} // namespace

std::unique_ptr<sla::RasterBase> PwmxArchive::create_raster() const
{
    sla::Resolution     res;
    sla::PixelDim       pxdim;
    std::array<bool, 2> mirror;

    double w  = m_cfg.display_width.getFloat();
    double h  = m_cfg.display_height.getFloat();
    auto   pw = size_t(m_cfg.display_pixels_x.getInt());
    auto   ph = size_t(m_cfg.display_pixels_y.getInt());

    mirror[X] = m_cfg.display_mirror_x.getBool();
    mirror[Y] = m_cfg.display_mirror_y.getBool();

    auto                         ro = m_cfg.display_orientation.getInt();
    sla::RasterBase::Orientation orientation =
        ro == sla::RasterBase::roPortrait ? sla::RasterBase::roPortrait :
                                            sla::RasterBase::roLandscape;

    if (orientation == sla::RasterBase::roPortrait) {
        std::swap(w, h);
        std::swap(pw, ph);
    }

    res   = sla::Resolution{pw, ph};
    pxdim = sla::PixelDim{w / pw, h / ph};
    sla::RasterBase::Trafo tr{orientation, mirror};

    double gamma = m_cfg.gamma_correction.getFloat();

    return sla::create_raster_grayscale_aa(res, pxdim, gamma, tr);
}

sla::RasterEncoder PwmxArchive::get_encoder() const
{
    return PWXRasterEncoder{};
}

// Endian safe write of little endian 32bit ints
static void pwmx_write_int32(std::ofstream &out, std::uint32_t val)
{
    const char i1 = (val & 0xFF);
    const char i2 = (val >> 8) & 0xFF;
    const char i3 = (val >> 16) & 0xFF;
    const char i4 = (val >> 24) & 0xFF;

    out.write((const char *) &i1, 1);
    out.write((const char *) &i2, 1);
    out.write((const char *) &i3, 1);
    out.write((const char *) &i4, 1);
}
static void pwmx_write_float(std::ofstream &out, std::float_t val)
{
    std::uint32_t *f = (std::uint32_t *) &val;
    pwmx_write_int32(out, *f);
}

static void pwmx_write_intro(std::ofstream &out, pwmx_format_intro_v516 &i, bool v516)
{
    out.write(TAG_INTRO, sizeof(i.tag));
    pwmx_write_int32(out, i.version);
    pwmx_write_int32(out, i.area_num);
    pwmx_write_int32(out, i.header_data_offset);
    pwmx_write_int32(out, i.intro24);
    pwmx_write_int32(out, i.preview_data_offset);
    pwmx_write_int32(out, i.intro32);
    pwmx_write_int32(out, i.layer_data_offset);
    if (v516) {
        pwmx_write_int32(out, i.extra_data_offset);
        pwmx_write_int32(out, i.machine_data_offset);
    } else {
        pwmx_write_int32(out, 0);
    }
    pwmx_write_int32(out, i.image_data_offset);
}

static void pwmx_write_header(std::ofstream &out, pwmx_format_header &h, bool v516)
{
    out.write(TAG_HEADER, sizeof(h.tag));
    pwmx_write_int32(out, h.payload_size);
    pwmx_write_float(out, h.pixel_size_um);
    pwmx_write_float(out, h.layer_height_mm);
    pwmx_write_float(out, h.exposure_time_s);
    pwmx_write_float(out, h.delay_before_exposure_s);
    pwmx_write_float(out, h.bottom_exposure_time_s);
    pwmx_write_float(out, h.bottom_layer_count);
    pwmx_write_float(out, h.lift_distance_mm);
    pwmx_write_float(out, h.lift_speed_mms);
    pwmx_write_float(out, h.retract_speed_mms);
    pwmx_write_float(out, h.volume_ml);
    pwmx_write_int32(out, h.antialiasing);
    pwmx_write_int32(out, h.res_x);
    pwmx_write_int32(out, h.res_y);
    pwmx_write_float(out, h.weight_g);
    pwmx_write_float(out, h.price);
    pwmx_write_int32(out, h.price_currency);
    pwmx_write_int32(out, h.per_layer_override);
    pwmx_write_int32(out, h.print_time_s);
    pwmx_write_int32(out, h.transition_layer_count);
    pwmx_write_int32(out, h.unknown);
    if (v516) {
        pwmx_write_int32(out, 0); //extra padding
    }
}

static void pwmx_write_preview(std::ofstream                    &out,
                               pwmx_format_preview              &p,
                               pwmx_format_preview_unknown_v516 &pu,
                               bool v516)
{
    out.write(TAG_PREVIEW, sizeof(p.tag));
    pwmx_write_int32(out, p.payload_size);
    pwmx_write_int32(out, p.preview_w);
    pwmx_write_int32(out, p.preview_dpi);
    pwmx_write_int32(out, p.preview_h);
    out.write((const char*) p.pixels, sizeof(p.pixels));

    if (v516) {
        int max = sizeof(pu) / sizeof(std::uint32_t);
        for (int i = 0; i < max; i++) {
            pwmx_write_int32(out, pu.unknown[i]);
        }
    }
}

static void pwmx_write_extra(std::ofstream &out, pwmx_format_extra &e)
{
    out.write(TAG_EXTRA, sizeof(e.tag));
    pwmx_write_int32(out, e.extra0);

    pwmx_write_int32(out, e.extra4);
    pwmx_write_float(out, e.lift_distance1_mm);
    pwmx_write_float(out, e.lift_speed1_mms);
    pwmx_write_float(out, e.retract_speed1_mms);
    pwmx_write_float(out, e.lift_distance2_mm);
    pwmx_write_float(out, e.lift_speed2_mms);
    pwmx_write_float(out, e.retract_speed2_mms);

    pwmx_write_int32(out, e.extra32);
    pwmx_write_float(out, e.lift_distance3_mm);
    pwmx_write_float(out, e.lift_speed3_mms);
    pwmx_write_float(out, e.retract_speed3_mms);
    pwmx_write_float(out, e.lift_distance4_mm);
    pwmx_write_float(out, e.lift_speed4_mms);
    pwmx_write_float(out, e.retract_speed4_mms);
}

static void pwmx_write_machine(std::ofstream &out, pwmx_format_machine &m)
{
    out.write(TAG_MACHINE, sizeof(m.tag));
    pwmx_write_int32(out, m.payload_size);

    out.write(m.name, sizeof(m.name));
    out.write(m.image_format, sizeof(m.image_format));
    pwmx_write_float(out, m.volume_x);
    pwmx_write_float(out, m.volume_y);
    pwmx_write_float(out, m.volume_z);
    pwmx_write_int32(out, m.version);
    pwmx_write_int32(out, m.machine140);
}

static void pwmx_write_layers_header(std::ofstream &out, pwmx_format_layers_header &h)
{
    out.write(TAG_LAYERS, sizeof(h.tag));
    pwmx_write_int32(out, h.payload_size);
    pwmx_write_int32(out, h.layer_count);
}

static void pwmx_write_layer(std::ofstream &out, pwmx_format_layer &l)
{
    pwmx_write_int32(out, l.image_offset);
    pwmx_write_int32(out, l.image_size);
    pwmx_write_float(out, l.lift_distance_mm);
    pwmx_write_float(out, l.lift_speed_mms);
    pwmx_write_float(out, l.exposure_time_s);
    pwmx_write_float(out, l.layer_height_mm);
    pwmx_write_float(out, l.layer44);
    pwmx_write_float(out, l.layer48);
}

void PwmxArchive::export_print(const std::string     fname,
                               const SLAPrint       &print,
                               const ThumbnailsList &thumbnails,
                               const std::string    &/*projectname*/)
{
    std::uint32_t layer_count = m_layers.size();

    pwmx_format_intro_v516    intro = {};
    pwmx_format_header        header = {};
    pwmx_format_preview       preview = {};
    pwmx_format_preview_unknown_v516 preview_unknown = {};
    pwmx_format_layers_header layers_header = {};
    pwmx_format_misc          misc = {};
    pwmx_format_extra         extra = {};
    pwmx_format_machine       machine = {};
    std::vector<uint8_t>      layer_images;
    std::uint32_t             image_offset;

    intro.version             = get_format_version(print);
    const bool is_v516        = (516 == intro.version);

    intro.area_num            = is_v516 ? 8 : 4;
    intro.header_data_offset  = is_v516 ? sizeof(pwmx_format_intro_v516) : sizeof(pwmx_format_intro);
    intro.preview_data_offset = intro.header_data_offset + sizeof(header) + (is_v516 ? 4 : 0);
    intro.layer_data_offset   = intro.preview_data_offset + sizeof(preview);
    if (is_v516) {
        //unknown data after preview bitmap
        intro.intro32 = intro.layer_data_offset;
        intro.layer_data_offset += sizeof(preview_unknown);
    }
    intro.image_data_offset = intro.layer_data_offset +
                              sizeof(layers_header) +
                              (sizeof(pwmx_format_layer) * layer_count);

    // extra section and machine section in between the layers and images
    if (is_v516) {
        intro.extra_data_offset = intro.image_data_offset;
        intro.machine_data_offset = intro.extra_data_offset +  sizeof(extra);
        intro.image_data_offset += sizeof(extra);
        intro.image_data_offset += sizeof(machine);
    }

    fill_header(header, misc, print, layer_count);
    fill_preview(preview, preview_unknown, misc, thumbnails, is_v516);
    if (is_v516) {
        fill_extra(extra, print);
        fill_machine(machine, print, intro.version);
    }

    try {
        // open the file and write the contents
        std::ofstream out;
        out.open(fname, std::ios::binary | std::ios::out | std::ios::trunc);
        pwmx_write_intro(out, intro, is_v516);
        pwmx_write_header(out, header, is_v516);
        pwmx_write_preview(out, preview, preview_unknown, is_v516);

        layers_header.payload_size = intro.image_data_offset - intro.layer_data_offset -
                        sizeof(layers_header.tag)  - sizeof(layers_header.payload_size);
        layers_header.layer_count = layer_count;
        pwmx_write_layers_header(out, layers_header);

        //layers
        layer_images.reserve(layer_count * LAYER_SIZE_ESTIMATE);
        image_offset = intro.image_data_offset;
        size_t i = 0;
        for (const sla::EncodedRaster &rst : m_layers) {
            pwmx_format_layer l;
            std::memset(&l, 0, sizeof(l));
            l.image_offset = image_offset;
            l.image_size = rst.size();
            if (i < header.bottom_layer_count) {
                l.exposure_time_s = header.bottom_exposure_time_s;
                l.layer_height_mm = misc.bottom_layer_height_mm;
                l.lift_distance_mm = misc.bottom_lift_distance_mm;
                l.lift_speed_mms = misc.bottom_lift_speed_mms;
            } else {
                l.exposure_time_s = header.exposure_time_s;
                l.layer_height_mm = header.layer_height_mm;
                l.lift_distance_mm = header.lift_distance_mm;
                l.lift_speed_mms = header.lift_speed_mms;
            }
            image_offset += l.image_size;
            pwmx_write_layer(out, l);
            // add the rle encoded layer image into the buffer
            const char* img_start = reinterpret_cast<const char*>(rst.data());
            const char* img_end = img_start + rst.size();
            std::copy(img_start, img_end, std::back_inserter(layer_images));
            i++;
        }
        if (is_v516) {
            pwmx_write_extra(out, extra);
            pwmx_write_machine(out, machine);
        }

        const char* img_buffer = reinterpret_cast<const char*>(layer_images.data());
        out.write(img_buffer, layer_images.size());
        out.close();
    } catch(std::exception& e) {
        BOOST_LOG_TRIVIAL(error) << e.what();
        // Rethrow the exception
        throw;
    }

}

} // namespace Slic3r
