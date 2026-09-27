// A4 sheets of AprilTag 36h11 for printing, no dependencies (Node >= 18):
//
//   node make_sheets.js [first_id] [count] [tag_cm] [out.pdf]     defaults: 0 24 6 tags_A4.pdf
//
// Codes and bit layout come from AprilRobotics/apriltag tag36h11.c (put next to this script, or it is
// downloaded). 6 per page (2 x 3), black square tag_cm wide; the robot's distance calibration assumes 6 cm.
// `node make_sheets.js --check` compares ids 0-5 with the tag36h11_0N.png files in this folder.
const fs = require("fs");
const path = require("path");
const zlib = require("zlib");

const HERE = __dirname;
const SRC = path.join(HERE, "tag36h11.c");
const SRC_URL = "https://raw.githubusercontent.com/AprilRobotics/apriltag/master/tag36h11.c";

async function family() {
  if (!fs.existsSync(SRC)) fs.writeFileSync(SRC, await (await fetch(SRC_URL)).text());
  const c = fs.readFileSync(SRC, "utf8");
  const codes = [...c.slice(c.indexOf("codedata")).matchAll(/0x([0-9a-fA-F]+)UL/g)].map((m) => BigInt("0x" + m[1]));
  const bx = [], by = [];
  for (const m of c.matchAll(/bit_([xy])\[(\d+)\] = (\d+);/g)) (m[1] === "x" ? bx : by)[+m[2]] = +m[3];
  if (codes.length !== 587 || bx.length !== 36) throw new Error("unexpected tag36h11.c format");
  return { codes, bx, by };
}

// 8 x 8 cells: black border ring + 6 x 6 data, true = white. apriltag_to_image() turned 180°, which is how
// OpenCV (cv2.aruco, what the robot runs) and the tag36h11_0N.png files draw it; detection works either way.
function grid(fam, id) {
  const g = Array.from({ length: 8 }, () => Array(8).fill(false));
  for (let i = 0; i < 36; i++) g[7 - fam.by[i]][7 - fam.bx[i]] = ((fam.codes[id] >> BigInt(35 - i)) & 1n) === 1n;
  return g;
}

function pdf(pages) {
  // pages: array of content-stream strings, A4 portrait, Helvetica as F1
  const objs = [];
  const add = (s) => objs.push(s) && objs.length;
  const font = add("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>");
  const pagesId = objs.length + 1 + pages.length * 2;
  const kids = pages.map((body) => {
    const content = add(`<< /Length ${Buffer.byteLength(body, "latin1")} >>\nstream\n${body}\nendstream`);
    return add(`<< /Type /Page /Parent ${pagesId} 0 R /MediaBox [0 0 595.28 841.89] ` +
      `/Resources << /Font << /F1 ${font} 0 R >> >> /Contents ${content} 0 R >>`);
  });
  add(`<< /Type /Pages /Kids [${kids.map((k) => k + " 0 R").join(" ")}] /Count ${kids.length} >>`);
  const catalog = add(`<< /Type /Catalog /Pages ${pagesId} 0 R >>`);
  let out = "%PDF-1.4\n", xref = [];
  objs.forEach((o, i) => { xref.push(out.length); out += `${i + 1} 0 obj\n${o}\nendobj\n`; });
  const at = out.length;
  out += `xref\n0 ${objs.length + 1}\n0000000000 65535 f \n` + xref.map((x) => String(x).padStart(10, "0") + " 00000 n \n").join("");
  out += `trailer\n<< /Size ${objs.length + 1} /Root ${catalog} 0 R >>\nstartxref\n${at}\n%%EOF\n`;
  return Buffer.from(out, "latin1");
}

const PT_CM = 72 / 2.54;
const f = (x) => x.toFixed(2);

function page(fam, ids, tagCm, pageNo, pageCount) {
  const W = 595.28, H = 841.89, top = 70, cellW = W / 2, cellH = (H - top - 20) / 3, s = tagCm * PT_CM, c = s / 8;
  let o = "";
  const text = (x, y, size, str) => { o += `BT /F1 ${size} Tf ${f(x)} ${f(y)} Td (${str}) Tj ET\n`; };
  text(36, H - 32, 11, `AprilTag 36h11, black square ${tagCm} cm  -  page ${pageNo}/${pageCount}`);
  text(36, H - 47, 9, `Print at 100 % / actual size, not "fit to page". Keep the white margin around each tag.`);
  text(36, H - 60, 9, "The bar on the right must measure exactly 5 cm:");
  o += `0 g ${f(W - 36 - 5 * PT_CM)} ${f(H - 60)} ${f(5 * PT_CM)} 6 re f\n`;
  o += "0.75 G 0.5 w [4 4] 0 d\n";  // light dashed cut lines
  o += `${f(cellW)} 20 m ${f(cellW)} ${f(H - top + 10)} l S\n`;
  for (let r = 1; r < 3; r++) o += `0 ${f(H - top - r * cellH)} m ${f(W)} ${f(H - top - r * cellH)} l S\n`;
  o += "[] 0 d\n";
  ids.forEach((id, k) => {
    const cx = (k % 2) * cellW + cellW / 2, cy = H - top - Math.floor(k / 2) * cellH - cellH / 2 + 10;
    const x0 = cx - s / 2, y0 = cy - s / 2, g = grid(fam, id);
    o += `0 g ${f(x0)} ${f(y0)} ${f(s)} ${f(s)} re f\n1 g\n`;
    const e = 0.4;  // grow each white cell into its white neighbours: no hairline seams, black cells untouched
    for (let r = 0; r < 8; r++) {
      for (let col = 0; col < 8; col++) {
        if (!g[r][col]) continue;
        const L = g[r][col - 1] ? e : 0, R = g[r][col + 1] ? e : 0, U = r > 0 && g[r - 1][col] ? e : 0, D = r < 7 && g[r + 1][col] ? e : 0;
        o += `${f(x0 + col * c - L)} ${f(y0 + (7 - r) * c - D)} ${f(c + L + R)} ${f(c + U + D)} re f\n`;
      }
    }
    o += "0 g\n";
    text(cx - 14, y0 - 24, 14, `id ${id}`);
  });
  return o;
}

// ---- --check: compare with the existing PNGs (the camera already detected those) ----
function readPng(file) {
  const b = fs.readFileSync(file);
  let p = 8, w, h, depth, type, idat = [];
  while (p < b.length) {
    const len = b.readUInt32BE(p), t = b.toString("latin1", p + 4, p + 8), d = b.subarray(p + 8, p + 8 + len);
    if (t === "IHDR") { w = d.readUInt32BE(0); h = d.readUInt32BE(4); depth = d[8]; type = d[9]; if (d[12]) throw new Error("interlaced"); }
    if (t === "IDAT") idat.push(d);
    p += 12 + len;
  }
  if (depth !== 8) throw new Error("need 8-bit PNG");
  const bpp = { 0: 1, 2: 3, 4: 2, 6: 4 }[type], raw = zlib.inflateSync(Buffer.concat(idat)), stride = w * bpp;
  const px = Buffer.alloc(h * stride);
  for (let y = 0; y < h; y++) {
    const ft = raw[y * (stride + 1)], line = raw.subarray(y * (stride + 1) + 1, (y + 1) * (stride + 1));
    for (let x = 0; x < stride; x++) {
      const a = x >= bpp ? px[y * stride + x - bpp] : 0, up = y ? px[(y - 1) * stride + x] : 0;
      const ul = y && x >= bpp ? px[(y - 1) * stride + x - bpp] : 0;
      const pa = Math.abs(up - ul), pb = Math.abs(a - ul), pc = Math.abs(a + up - 2 * ul);
      const pred = [0, a, up, (a + up) >> 1, pa <= pb && pa <= pc ? a : pb <= pc ? up : ul][ft];
      px[y * stride + x] = (line[x] + pred) & 255;
    }
  }
  return { w, h, lum: (x, y) => px[y * stride + x * bpp] };  // first channel is enough for black/white
}

function pngGrid(file) {
  const im = readPng(file);
  // find the black square's bounding box, then sample the 8 x 8 cell centres
  let x0 = im.w, y0 = im.h, x1 = -1, y1 = -1;
  for (let y = 0; y < im.h; y++) for (let x = 0; x < im.w; x++) if (im.lum(x, y) < 128) {
    x0 = Math.min(x0, x); y0 = Math.min(y0, y); x1 = Math.max(x1, x); y1 = Math.max(y1, y);
  }
  const cw = (x1 - x0 + 1) / 8, ch = (y1 - y0 + 1) / 8;
  return Array.from({ length: 8 }, (_, r) => Array.from({ length: 8 }, (_, c) =>
    im.lum(Math.floor(x0 + (c + 0.5) * cw), Math.floor(y0 + (r + 0.5) * ch)) >= 128));
}

(async () => {
  const fam = await family();
  if (process.argv[2] === "--check") {
    let bad = 0;
    for (let id = 0; id <= 5; id++) {
      const file = path.join(HERE, `tag36h11_${String(id).padStart(2, "0")}.png`);
      if (!fs.existsSync(file)) continue;
      const same = JSON.stringify(pngGrid(file)) === JSON.stringify(grid(fam, id));
      console.log(`id ${id}: ${same ? "matches" : "DIFFERENT"} ${path.basename(file)}`);
      bad += !same;
    }
    process.exit(bad ? 1 : 0);
  }
  const [first = 0, count = 24, tagCm = 6, out = "tags_A4.pdf"] = process.argv.slice(2);
  const ids = Array.from({ length: +count }, (_, i) => +first + i);
  const chunks = [];
  for (let i = 0; i < ids.length; i += 6) chunks.push(ids.slice(i, i + 6));
  const file = path.resolve(HERE, out);
  fs.writeFileSync(file, pdf(chunks.map((c, i) => page(fam, c, +tagCm, i + 1, chunks.length))));
  console.log(`${ids.length} tags (id ${ids[0]}-${ids.at(-1)}), ${chunks.length} pages -> ${file}`);
})();
