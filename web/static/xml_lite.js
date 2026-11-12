// Tiny XML reader for GPX/TCX (works in the browser and in node, so the tests run the same code as the page).
// Builds { name (local name, no prefix), attrs, children, text } where text is the character data before the first
// child element (comments skipped, CDATA and entities decoded), like ElementTree's .text. Throws on malformed XML.
import { pyStrip } from './track_values.js';

const TOKEN = /<!--[\s\S]*?-->|<!\[CDATA\[([\s\S]*?)\]\]>|<\?[\s\S]*?\?>|<!DOCTYPE[^>[]*(?:\[[\s\S]*?\])?\s*>|<(\/?)([A-Za-z_][\w:.-]*)((?:\s+[A-Za-z_][\w:.-]*\s*=\s*(?:"[^"<]*"|'[^'<]*'))*)\s*(\/?)>|([^<]+)|(<)/g;
const ATTR = /([A-Za-z_][\w:.-]*)\s*=\s*(?:"([^"]*)"|'([^']*)')/g;
const XML_DECL = /^<\?xml\s+version\s*=\s*(["'])[\w.:-]*\1(\s+encoding\s*=\s*(["'])[A-Za-z][\w.-]*\3)?(\s+standalone\s*=\s*(["'])(yes|no)\5)?\s*\?>$/;
const NAMED = { amp: '&', lt: '<', gt: '>', quot: '"', apos: "'" };

function decode(s) {
  return s.replace(/&(#x[0-9a-fA-F]+|#[0-9]+|[A-Za-z_][\w.-]*);|&/g, (m, e) => {
    if (e === undefined) throw new Error('Not valid XML: "&" not starting an entity');
    if (e[0] === '#') {
      const c = e[1] === 'x' ? parseInt(e.slice(2), 16) : parseInt(e.slice(1), 10);
      if (!(c === 9 || c === 10 || c === 13 || (c >= 0x20 && c <= 0xd7ff) || (c >= 0xe000 && c <= 0xfffd) || (c >= 0x10000 && c <= 0x10ffff)))
        throw new Error(`Not valid XML: bad character reference ${m}`);
      return String.fromCodePoint(c);
    }
    if (e in NAMED) return NAMED[e];
    throw new Error(`Not valid XML: undefined entity ${m}`);
  });
}

// Namespace prefixes in scope must be declared (xmlns:p="..."), like expat in ElementTree.
function checkPrefixes(el, attrNames, scope) {
  const declared = new Set(scope);
  for (const n of attrNames) if (n.startsWith('xmlns:')) declared.add(n.slice(6));
  const prefixOf = n => (n.includes(':') ? n.slice(0, n.indexOf(':')) : null);
  for (const n of [el.qname, ...attrNames]) {
    const p = prefixOf(n);
    if (p !== null && p !== 'xml' && p !== 'xmlns' && !declared.has(p)) throw new Error(`Not valid XML: unbound prefix ${p}`);
  }
  return declared;
}

const local = name => name.slice(name.lastIndexOf(':') + 1);

export function parseXml(text) {
  const src = pyStrip(String(text).replace(/^\ufeff+/, ''));
  const stack = [], top = () => stack[stack.length - 1];
  let root = null, m;
  TOKEN.lastIndex = 0;
  while ((m = TOKEN.exec(src))) {
    const [all, cdata, close, tag, attrs, selfClose, chars, stray] = m;
    if (stray) throw new Error('Not valid XML: unexpected "<"');
    if (cdata !== undefined || chars !== undefined) {
      const el = top();
      const t = cdata !== undefined ? cdata : decode(chars);
      if (!el) { if (cdata !== undefined || /[^ \t\r\n]/.test(chars)) throw new Error('Not valid XML: text outside the root element'); continue; }
      if (!el.children.length) el.text += t;
      continue;
    }
    if (!tag) {   // comment, processing instruction, doctype; the XML declaration must be first and well formed
      if (/^<\?xml[\s?]/i.test(all) && (m.index !== 0 || !XML_DECL.test(all))) throw new Error('Not valid XML: bad XML declaration');
      if (all.startsWith('<!--') && all.slice(4, -3).includes('--')) throw new Error('Not valid XML: "--" inside a comment');
      if (all.startsWith('<?') && !/^<\?[A-Za-z_][\w.-]*(\s[\s\S]*)?\?>$/.test(all)) throw new Error('Not valid XML: bad processing instruction');
      continue;
    }
    if (close) {
      const el = stack.pop();
      if (!el || el.qname !== tag) throw new Error(`Not valid XML: mismatched </${tag}>`);
      continue;
    }
    const el = { name: local(tag), qname: tag, attrs: {}, children: [], text: '' };
    const found = [...attrs.matchAll(ATTR)];
    for (const a of found) {
      if (a[1] in el.attrs) throw new Error(`Not valid XML: duplicate attribute ${a[1]}`);
      el.attrs[a[1]] = decode(a[2] !== undefined ? a[2] : a[3]).replace(/[\t\n\r]/g, ' ');
    }
    el.scope = checkPrefixes(el, found.map(a => a[1]), top() ? top().scope : []);
    if (top()) top().children.push(el);
    else if (root) throw new Error('Not valid XML: more than one root element');
    else root = el;
    if (!selfClose) stack.push(el);
  }
  if (!root) throw new Error('Not valid XML: no element found');
  if (stack.length) throw new Error(`Not valid XML: <${top().qname}> is not closed`);
  return root;
}

export const children = (el, name) => (el ? el.children.filter(c => c.name === name) : []);
export const child = (el, name) => (el ? el.children.find(c => c.name === name) || null : null);
export function* iter(el) { yield el; for (const c of el.children) yield* iter(c); }
