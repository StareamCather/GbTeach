"""Small escaped Markdown renderer for Qt 5.12 (no web server or network assets)."""

import html
import re


STYLE = """
body { color:#243247; background:#ffffff; font-family:'Noto Sans CJK SC','Microsoft YaHei',sans-serif;
       font-size:14px; line-height:1.65; margin:24px; }
h1 { color:#123052; font-size:27px; margin-bottom:16px; }
h2 { color:#17639a; font-size:21px; margin-top:27px; }
h3 { color:#204862; font-size:17px; margin-top:22px; }
p,li { margin-top:8px; margin-bottom:8px; }
pre { background:#eef3f9; padding:14px; border:1px solid #dbe4ef; white-space:pre-wrap; }
code { font-family:'DejaVu Sans Mono',monospace; color:#9c4b0b; }
blockquote { border-left:4px solid #f2a24a; margin-left:4px; padding-left:15px; color:#52667f; }
table { border-collapse:collapse; margin-top:14px; margin-bottom:14px; }
td,th { border:1px solid #d8e2ee; padding:8px; text-align:left; }
th { background:#edf4fb; color:#173a5f; } a { color:#146ba5; }
"""


def inline(text):
    escaped = html.escape(text)
    escaped = re.sub(r"`([^`]+)`", r"<code>\1</code>", escaped)
    escaped = re.sub(r"\*\*([^*]+)\*\*", r"<b>\1</b>", escaped)
    def link(match):
        label, url = match.group(1), html.unescape(match.group(2))
        if not re.match(r"^https?://", url, re.I):
            return label + " (" + html.escape(url) + ")"
        return '<a href="{}">{}</a>'.format(html.escape(url, quote=True), label)
    return re.sub(r"\[([^\]]+)\]\(([^)]+)\)", link, escaped)


def render(text):
    lines = text.splitlines()
    output, paragraph, code = [], [], []
    in_code, listing, in_table = False, None, False
    def flush():
        if paragraph:
            output.append("<p>" + "<br>".join(inline(x) for x in paragraph) + "</p>")
            paragraph.clear()
    def end_list():
        nonlocal listing
        if listing:
            output.append("</" + listing + ">")
            listing = None
    for line in lines:
        if line.strip().startswith("```"):
            flush(); end_list()
            if in_code:
                output.append("<pre><code>" + html.escape("\n".join(code)) + "</code></pre>")
                code.clear()
            in_code = not in_code
            continue
        if in_code:
            code.append(line)
            continue
        if line.strip().startswith("|") and line.strip().endswith("|"):
            flush(); end_list()
            cells = [x.strip() for x in line.strip().strip("|").split("|")]
            if all(re.fullmatch(r":?-{2,}:?", x) for x in cells):
                continue
            first = not in_table
            if first:
                output.append("<table>"); in_table = True
            tag = "th" if first else "td"
            output.append("<tr>" + "".join("<{}>{}</{}>".format(tag, inline(x), tag) for x in cells) + "</tr>")
            continue
        if in_table:
            output.append("</table>"); in_table = False
        heading = re.match(r"^(#{1,4})\s+(.+)$", line)
        item = re.match(r"^\s*(?:[-*]\s+|\d+[.)]\s+)(.+)$", line)
        if heading:
            flush(); end_list()
            size = len(heading.group(1))
            output.append("<h{0}>{1}</h{0}>".format(size, inline(heading.group(2))))
        elif item:
            flush()
            kind = "ol" if re.match(r"^\s*\d", line) else "ul"
            if listing != kind:
                end_list(); output.append("<" + kind + ">"); listing = kind
            output.append("<li>" + inline(item.group(1)) + "</li>")
        elif not line.strip():
            flush(); end_list()
        elif line.startswith(">"):
            flush(); end_list(); output.append("<blockquote>" + inline(line.lstrip("> ")) + "</blockquote>")
        elif re.fullmatch(r"\s*[-*_]{3,}\s*", line):
            flush(); end_list(); output.append("<hr>")
        else:
            end_list(); paragraph.append(line)
    flush(); end_list()
    if in_table:
        output.append("</table>")
    if code:
        output.append("<pre>" + html.escape("\n".join(code)) + "</pre>")
    return "<html><head><style>" + STYLE + "</style></head><body>" + "\n".join(output) + "</body></html>"
