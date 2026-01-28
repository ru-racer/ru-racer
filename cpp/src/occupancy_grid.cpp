#include "ru_racer/occupancy_grid.hpp"

#include <cctype>
#include <cstdint>
#include <fstream>
#include <sstream>

namespace ru_racer {

static bool readNextToken(std::istream& in, std::string& tok) {
  tok.clear();
  char c;
  while (in.get(c)) {
    if (c == '#') {
      std::string dummy;
      std::getline(in, dummy);
      continue;
    }
    if (!std::isspace(static_cast<unsigned char>(c))) {
      tok.push_back(c);
      break;
    }
  }
  if (tok.empty()) return false;
  while (in.get(c)) {
    if (std::isspace(static_cast<unsigned char>(c))) break;
    tok.push_back(c);
  }
  return true;
}

bool OccupancyGrid::loadPgm(const std::filesystem::path& path, std::string* err) {
  std::ifstream in(path, std::ios::binary);
  if (!in.good()) {
    if (err) *err = "Failed to open: " + path.string();
    return false;
  }

  std::string magic;
  if (!readNextToken(in, magic)) {
    if (err) *err = "Invalid PGM header (missing magic)";
    return false;
  }
  if (magic != "P2" && magic != "P5") {
    if (err) *err = "Unsupported PGM type: " + magic + " (expected P2 or P5)";
    return false;
  }

  std::string tok;
  if (!readNextToken(in, tok)) return false;
  width = std::stoi(tok);
  if (!readNextToken(in, tok)) return false;
  height = std::stoi(tok);
  if (!readNextToken(in, tok)) return false;
  const int maxv = std::stoi(tok);
  if (width <= 0 || height <= 0 || maxv <= 0) {
    if (err) *err = "Invalid PGM dimensions/max value";
    return false;
  }

  data.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);

  if (magic == "P2") {
    for (int i = 0; i < width * height; ++i) {
      if (!readNextToken(in, tok)) {
        if (err) *err = "Unexpected EOF reading P2 pixels";
        return false;
      }
      int v = std::stoi(tok);
      if (v < 0) v = 0;
      if (v > maxv) v = maxv;
      const int scaled = (maxv == 255) ? v : static_cast<int>((static_cast<double>(v) / static_cast<double>(maxv)) * 255.0);
      data[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(scaled);
    }
    return true;
  }

  // P5 binary: consume one whitespace after maxv (if any) then read bytes.
  // readNextToken already left stream right after maxv token; PGM spec allows single whitespace.
  // We'll just read one char if it's whitespace.
  char c;
  if (in.get(c)) {
    if (!std::isspace(static_cast<unsigned char>(c))) in.unget();
  }

  std::vector<std::uint8_t> raw(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
  in.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(raw.size()));
  if (in.gcount() != static_cast<std::streamsize>(raw.size())) {
    if (err) *err = "Unexpected EOF reading P5 pixels";
    return false;
  }
  if (maxv == 255) {
    data = std::move(raw);
    return true;
  }

  for (std::size_t i = 0; i < raw.size(); ++i) {
    const int v = raw[i];
    const int scaled = static_cast<int>((static_cast<double>(v) / static_cast<double>(maxv)) * 255.0);
    data[i] = static_cast<std::uint8_t>(scaled);
  }
  return true;
}

static std::uint16_t readU16LE(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0]) | (static_cast<std::uint16_t>(p[1]) << 8);
}
static std::uint32_t readU32LE(const std::uint8_t* p) {
  return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) | (static_cast<std::uint32_t>(p[2]) << 16) |
         (static_cast<std::uint32_t>(p[3]) << 24);
}
static std::int32_t readI32LE(const std::uint8_t* p) { return static_cast<std::int32_t>(readU32LE(p)); }

bool OccupancyGrid::loadBmp(const std::filesystem::path& path, std::string* err) {
  std::ifstream in(path, std::ios::binary);
  if (!in.good()) {
    if (err) *err = "Failed to open: " + path.string();
    return false;
  }

  // Read entire file (BMPs used here are small enough).
  in.seekg(0, std::ios::end);
  const auto sz = static_cast<std::size_t>(in.tellg());
  in.seekg(0, std::ios::beg);
  std::vector<std::uint8_t> buf(sz);
  in.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
  if (static_cast<std::size_t>(in.gcount()) != buf.size()) {
    if (err) *err = "Failed to read BMP data";
    return false;
  }

  if (buf.size() < 54) {
    if (err) *err = "BMP too small";
    return false;
  }
  if (buf[0] != 'B' || buf[1] != 'M') {
    if (err) *err = "Not a BMP file";
    return false;
  }

  const std::uint32_t pixel_offset = readU32LE(&buf[10]);
  const std::uint32_t dib_size = readU32LE(&buf[14]);
  if (dib_size < 40) {
    if (err) *err = "Unsupported BMP DIB header size";
    return false;
  }

  const std::int32_t w = readI32LE(&buf[18]);
  const std::int32_t h_raw = readI32LE(&buf[22]);
  const bool top_down = h_raw < 0;
  const std::int32_t h = top_down ? -h_raw : h_raw;
  const std::uint16_t planes = readU16LE(&buf[26]);
  const std::uint16_t bpp = readU16LE(&buf[28]);
  const std::uint32_t compression = readU32LE(&buf[30]);
  if (planes != 1 || (bpp != 8 && bpp != 24) || compression != 0) {
    if (err) *err = "Unsupported BMP format (expected uncompressed 8-bit or 24-bit)";
    return false;
  }
  if (w <= 0 || h <= 0) {
    if (err) *err = "Invalid BMP dimensions";
    return false;
  }
  if (pixel_offset >= buf.size()) {
    if (err) *err = "Invalid BMP pixel offset";
    return false;
  }

  width = static_cast<int>(w);
  height = static_cast<int>(h);
  data.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);

  // Palette for 8-bit
  std::vector<std::uint8_t> palette; // BGRA entries
  if (bpp == 8) {
    const std::uint32_t palette_offset = 14 + dib_size;
    const std::uint32_t palette_bytes = pixel_offset - palette_offset;
    if (palette_offset >= buf.size() || pixel_offset > buf.size() || palette_bytes < 4) {
      if (err) *err = "Invalid BMP palette region";
      return false;
    }
    palette.assign(buf.begin() + palette_offset, buf.begin() + pixel_offset);
  }

  const int bytes_per_pixel = (bpp == 24) ? 3 : 1;
  const std::size_t row_stride_raw = static_cast<std::size_t>(width) * static_cast<std::size_t>(bytes_per_pixel);
  const std::size_t row_stride = (row_stride_raw + 3u) & ~3u; // 4-byte aligned

  for (int row = 0; row < height; ++row) {
    const int src_row = top_down ? row : (height - 1 - row);
    const std::size_t src_off = static_cast<std::size_t>(pixel_offset) + static_cast<std::size_t>(src_row) * row_stride;
    if (src_off + row_stride_raw > buf.size()) {
      if (err) *err = "BMP pixel data out of bounds";
      return false;
    }
    for (int col = 0; col < width; ++col) {
      std::uint8_t gray = 0;
      if (bpp == 24) {
        const std::size_t p = src_off + static_cast<std::size_t>(col) * 3u;
        const std::uint8_t B = buf[p + 0];
        const std::uint8_t G = buf[p + 1];
        const std::uint8_t R = buf[p + 2];
        gray = static_cast<std::uint8_t>((static_cast<int>(R) * 30 + static_cast<int>(G) * 59 + static_cast<int>(B) * 11) / 100);
      } else {
        const std::size_t p = src_off + static_cast<std::size_t>(col);
        const std::uint8_t idx = buf[p];
        // Palette entries are BGRA (4 bytes). Use luminance from palette if present.
        if (!palette.empty() && static_cast<std::size_t>(idx) * 4u + 3u < palette.size()) {
          const std::uint8_t B = palette[static_cast<std::size_t>(idx) * 4u + 0u];
          const std::uint8_t G = palette[static_cast<std::size_t>(idx) * 4u + 1u];
          const std::uint8_t R = palette[static_cast<std::size_t>(idx) * 4u + 2u];
          gray = static_cast<std::uint8_t>((static_cast<int>(R) * 30 + static_cast<int>(G) * 59 + static_cast<int>(B) * 11) / 100);
        } else {
          gray = idx;
        }
      }
      data[static_cast<std::size_t>(row) * static_cast<std::size_t>(width) + static_cast<std::size_t>(col)] = gray;
    }
  }
  return true;
}

bool OccupancyGrid::load(const std::filesystem::path& path, std::string* err) {
  const auto ext = path.extension().string();
  std::string lower;
  lower.reserve(ext.size());
  for (char c : ext) lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  if (lower == ".pgm") return loadPgm(path, err);
  if (lower == ".bmp") return loadBmp(path, err);
  if (err) *err = "Unsupported map extension: " + lower + " (supported: .pgm, .bmp)";
  return false;
}

bool OccupancyGrid::isFreeWorld(double x_m, double y_m) const {
  // mimic MATLAB:
  // X = max(1,ceil(x*100+2.819*100));
  // Y = max(1,ceil(3.581*100-y*100));
  // and then clamp to image bounds. MATLAB is 1-based, we are 0-based.
  double px_d = 0.0;
  double py_d = 0.0;
  if (use_affine_transform) {
    px_d = std::ceil(a11 * x_m + a12 * y_m + b1);
    py_d = std::ceil(a21 * x_m + a22 * y_m + b2);
  } else {
    px_d = std::ceil(x_m * pixels_per_meter + origin_x_px);
    py_d = std::ceil(origin_y_px - y_m * pixels_per_meter);
  }
  int px = static_cast<int>(px_d) - 1;
  int py = static_cast<int>(py_d) - 1;
  if (px < 0) px = 0;
  if (py < 0) py = 0;
  if (px >= width) px = width - 1;
  if (py >= height) py = height - 1;
  return isFreePixel(px, py);
}

} // namespace ru_racer

