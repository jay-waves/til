/*
* 数学宏： theorem, lemma, corollary, definition, proof
* 样式宏：tufte + note, 
* 接收的系统输入：theme=dark/light, layout=landscape/portrait
*/

#import "@preview/ctheorems:1.1.3": *
#import "@preview/physica:0.9.8" as physica

#assert(
  sys.version >= version(0, 15, 0),
  message: "This theme requires Typst 0.15.0 or newer.",
)

#let main-fonts = (
  "EB Garamond",
  "Source Han Serif",
  "Libertinus Serif",
  "Libertinus Sans",
)

#let heading-fonts = (
  "EB Garamond",
  "Source Han Sans",
  "Microsoft YaHei",
  "SimHei",
  "Libertinus Sans",
)

#let code-fonts = (
  "Fira Code",
  "Cascadia Mono",
  "DejaVu Sans Mono",
  "Courier New",
)

#let preview-info = json(
  bytes(sys.inputs.at("x-preview", default: "{}"))
)

#let theme = sys.inputs.at(
  "theme",
  default: preview-info.at("theme", default: "light"),
)

#let layout = sys.inputs.at("layout", default: "landscape")
#assert(
  layout in ("portrait", "landscape"),
  message: "The layout input must be either \"portrait\" or \"landscape\".",
)

#let dark = theme == "dark"
#let landscape = layout == "landscape"
#let page-columns = if landscape { 2 } else { 1 }
#let column-gutter = 4%

#let fg = if dark { rgb("#f0f6fc") } else { rgb("#1f2328") }
#let bg = if dark { rgb("#212830") } else { white }
#let muted = if dark { rgb("#9198a1") } else { rgb("#59636e") }
#let border = if dark { rgb("#3d444d") } else { rgb("#d1d9e0") }
#let border-muted = if dark { rgb("#3d444db3") } else { rgb("#d1d9e0b3") }
#let accent = if dark { rgb("#478be6") } else { rgb("#0969da") }
#let code-fg = if dark { rgb("#c9d1d9") } else { rgb("#24292f") }
#let pre-bg = if dark { rgb("#262c36") } else { rgb("#f6f8fa") }
#let shadow = if dark { rgb("#00000066") } else { rgb("#1f232826") }
#let cjk-text = regex("\p{Han}+")

#let note-size = 7pt
#let note-body-ratio = 3fr
#let note-aside-ratio = 1fr
#let note-gutter = 4%

#let note(body, aside) = {
  block(
    width: 100%,
    breakable: true,
    grid(
      columns: (note-body-ratio, note-aside-ratio),
      column-gutter: note-gutter,
      align: top + left,
      block(width: 100%, body),
      block(
        width: 100%,
        inset: (x: 0.75em, y: 0.6em),
        radius: 5pt,
        fill: pre-bg,
        stroke: (
          top: 0.35pt + border-muted,
          left: 0.35pt + border-muted,
          right: 1.2pt + shadow,
          bottom: 1.2pt + shadow,
        ),
        text(size: note-size, aside),
      ),
    ),
  )
}

#let theorem = thmbox("theorem", "定理", titlefmt: strong, fill: pre-bg, stroke: 0.4pt + border, radius: 3pt)
#let lemma = thmbox("lemma", "引理", titlefmt: strong, fill: pre-bg, stroke: 0.4pt + border, radius: 3pt)
#let corollary = thmplain("corollary", "推论", titlefmt: strong)
#let definition = thmbox("definition", "定义", fill: pre-bg, stroke: 0.4pt + border, radius: 3pt)
#let proof-env = thmproof("proof", "证明", titlefmt: strong, inset: (top: 0em, left: 0pt, bottom: 0em, right: 0pt))
#let proof(..args, body) = {
  proof-env(..args, body)
  linebreak()
}

#let meta(
  subtitle: none,
  source: none,
  revised: none,
  copyright: none,
  license: none,
  code: none,
  tags: none,
) = {
  let fields = (
    ("Subtitle", subtitle),
    ("Source", source),
    ("Revised", revised),
    ("Copyright", copyright),
    ("License", license),
    ("Code", code),
    ("Tags", tags),
  ).filter(field => field.at(1) != none)
  let value = item => if type(item) == array { item.join(", ") } else { item }

  if fields.len() > 0 {
    block(
      width: 100%,
      inset: (left: 0.8em, y: 0.45em),
      stroke: (left: 2pt + accent),
      text(font: main-fonts, size: 8.5pt, fill: muted)[
        #grid(
          columns: (6.5em, 1fr),
          row-gutter: 0.25em,
          column-gutter: 0.8em,
          ..fields.map(field => (
            text(fill: accent, weight: "bold", upper(field.at(0))),
            value(field.at(1)),
          )).flatten(),
        )
      ],
    )
    v(1.1em)
  }
}

#let template(
  title: none,
  abstract: none,
  subtitle: none,
  authors: (),
  date: datetime.today(),
  body,
) = {
  let abstract = if abstract == none { subtitle } else { abstract }

  set page(
    paper: "a4",
    flipped: landscape,
    columns: page-columns,
    fill: bg,
    margin: (y: 3em, x: 2.5em),
    foreground: if landscape {
      place(
        center + horizon,
        rect(
          width: 0.55pt,
          height: 100% - 6em,
          fill: border-muted,
        ),
      )
    },
    header: context {
      if here().page() != 1 {
        set text(
          font: main-fonts,
          fill: muted,
          weight: "bold",
          size: 7pt,
          number-type: "old-style",
          number-width: "tabular",
        )
        place(right, dy: 3.5em)[
          #upper(title) #h(1em) #text(size: 11pt, counter(page).display())
        ]
      }
    },
  )
  set columns(gutter: column-gutter)

  show raw: it => {
    if it.block {
      v(0.8em)
      block(
        width: 100%,
        inset: 0.75em,
        radius: 3pt,
        fill: pre-bg,
        stroke: 0.4pt + border,
        text(font: code-fonts, weight: "regular", size: 7pt, fill: code-fg, it),
      )
      v(0.8em)
    } else {
      text(font: code-fonts, weight: "regular", fill: code-fg, it)
    }
  }

  show link: set text(fill: accent)
  show link: underline
  show strong: set text(weight: "bold")

  let has-authors = authors != () and authors.len() > 0
  if title != none or abstract != none or subtitle != none or has-authors {
    block(
      width: 100%,
      inset: 0pt,
      radius: 4pt,
      text(font: main-fonts, weight: "medium", size: 9pt, fill: fg)[
        #if title != none [#text(size: 12pt, upper(title))]

        #if has-authors [#upper(authors.join(", ", last: " and "))]

        #if abstract != none [
          #pad(
            x: 5em,
            block(abstract),
          )
        ]
      ],
    )
  }

  set heading(numbering: "1.1")
  show heading: it => context {
    let levels = counter(heading).get()
    let is-ack = it.body in ([Acknowledgment], [Acknowledgement])

    if it.level == 1 [
      #set text(font: heading-fonts, if is-ack { 9pt } else { 14pt }, weight: "bold", fill: accent)
      #v(32pt, weak: true)
      #if it.numbering != none and not is-ack {
        text(fill: accent)[#numbering("1.1", ..levels).]
        h(7pt, weak: true)
      }
      #it.body
      #v(5pt, weak: true)
      #line(length: 100%, stroke: 0.35pt + border-muted)
      #v(12pt, weak: true)
    ] else if it.level == 2 [
      #set par(first-line-indent: 0pt)
      #set text(font: heading-fonts, size: 11.5pt, weight: "bold", fill: accent)
      #v(18pt, weak: true)
      #if it.numbering != none {
        text(fill: accent)[#numbering("1.1", ..levels).]
        h(7pt, weak: true)
      }
      #it.body
      #v(12pt, weak: true)
    ] else [
      #set text(font: heading-fonts, size: 9.5pt, weight: "bold", fill: accent)
      #if it.numbering != none {
        text(fill: accent)[#numbering("1.1", ..levels). ]
      }
      #it.body
      #text(fill: accent)[:]
      #v(12pt, weak: true)
    ]
  }

  set text(
    font: main-fonts,
    fill: fg,
    weight: "regular",
    size: 9.3pt,
    number-type: "old-style",
    number-width: "tabular",
  )
  set par(leading: 0.86em)
  show figure: it => {
    v(1.1em)
    it
    v(1.1em)
  }
  show figure.caption: set text(size: 7pt)
  show image: set block(above: 1.1em, below: 1.1em)
  show cjk-text: set text(size: 0.9em, tracking: 0.05em)
  show: thmrules.with(qed-symbol: $square$)

  body
}

#let tufte(body) = template(body)
