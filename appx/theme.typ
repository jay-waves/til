// based on https://github.com/fredguth/tufte-typst

#assert(
  sys.version >= version(0, 15, 0),
  message: "This theme requires Typst 0.15.0 or newer.",
)

#let main-fonts = (
  "EB Garamond",
  "LXGW WenKai Lite",
  "Libertinus Serif",
  "Libertinus Sans",
)

#let code-fonts = (
  "Fira Code",
  "Cascadia Mono",
  "DejaVu Sans Mono",
  "Courier New",
)

#let accent = rgb("#2563a8")
#let latin-text = regex("[A-Za-z][A-Za-z0-9'’.,:;!?()/-]*")

#let sidenote(content) = {
  place(
    dx: 47.5em,
    block(
      breakable: false,
      width: 13em,
      content,
    ),
  )
}

#let note(body) = sidenote(text(size: 8.5pt, body))

#let meta(body) = {
  block(
    width: 100%,
    inset: (left: 0.8em, y: 0.45em),
    stroke: (left: 2pt + accent),
    text(font: main-fonts, size: 8.5pt, fill: rgb("#4b5563"))[
      #set list(marker: [–])
      #body
    ],
  )
  v(1.1em)
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
    margin: (y: 6em, left: 4em, right: 16em),
    header: context {
      if here().page() != 1 {
        set text(
          font: main-fonts,
          weight: "semibold",
          size: 7pt,
          tracking: 1.1pt,
          number-type: "old-style",
          number-width: "tabular",
        )
        place(right, dy: 6em, dx: 16em)[
          #upper(title) #h(1em) #text(size: 11pt, counter(page).display())
        ]
      }
    },
  )

  show raw: it => {
    if it.block {
      v(0.8em)
      block(
        width: 100%,
        inset: 0.75em,
        radius: 3pt,
        fill: rgb("#fafbfc"),
        stroke: 0.4pt + rgb("#eaeef2"),
        text(font: code-fonts, weight: "regular", size: 7pt, it),
      )
      v(0.8em)
    } else {
      it
    }
  }

  show link: set text(fill: accent)
  show link: underline

  let has-authors = authors != () and authors.len() > 0
  if title != none or abstract != none or subtitle != none or has-authors {
    block(
      width: 100% + 14em - 4em,
      inset: 0pt,
      radius: 4pt,
      text(font: main-fonts, weight: "medium", size: 9pt, tracking: 2pt)[
        #if title != none [#text(size: 12pt, upper(title))]

        #if has-authors [#upper(authors.join(", ", last: " and "))]

        #if abstract != none [
          #pad(
            x: 5em,
            block(text(tracking: 0pt, abstract)),
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
      #set text(if is-ack { 9pt } else { 14pt }, weight: "semibold", fill: accent)
      #v(32pt, weak: true)
      #if it.numbering != none and not is-ack {
        text(fill: accent)[#numbering("1.1", ..levels).]
        h(7pt, weak: true)
      }
      #it.body
      #v(5pt, weak: true)
      #line(length: 100%, stroke: 0.35pt + rgb("#d7e1ee"))
      #v(12pt, weak: true)
    ] else if it.level == 2 [
      #set par(first-line-indent: 0pt)
      #set text(size: 11.5pt, weight: "semibold", fill: accent)
      #v(18pt, weak: true)
      #if it.numbering != none {
        text(fill: accent)[#numbering("1.1", ..levels).]
        h(7pt, weak: true)
      }
      #it.body
      #v(9pt, weak: true)
    ] else [
      #set text(size: 9.5pt, weight: "semibold", fill: accent)
      #if it.numbering != none {
        text(fill: accent)[#numbering("1.1", ..levels). ]
      }
      #it.body
      #text(fill: accent)[:]
    ]
  }

  set text(
    font: main-fonts,
    weight: "medium",
    size: 9pt,
    tracking: 0pt,
    number-type: "old-style",
    number-width: "tabular",
  )
  set par(leading: 0.72em)
  show figure: it => {
    v(1.1em)
    it
    v(1.1em)
  }
  show figure.caption: set text(size: 7pt)
  show image: set block(above: 1.1em, below: 1.1em)
  show latin-text: set text(font: "EB Garamond", size: 1.1em)

  body
}

#let tufte(body) = template(body)
