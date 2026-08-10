/*
* arguments (sys-input): theme=dark/light, layout=landscape/portrait
*/

#import "@preview/ctheorems:1.1.3": *
#import "@preview/physica:0.9.8" as physica
#import "@preview/merman:0.1.0": mermaid as merman

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

#let fsize = (
  tiny: 5.6pt,
  small: 6.8pt,
  body: 7.4pt,
  h1: 11.2pt,
  h2: 9.2pt,
  h3: 7.6pt,
)

#let spacing = (
    xs: 5pt,
    sm: 8pt,
    md: 12pt,
    lg: 18pt,
    xl: 32pt,
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

#let fg = if dark { rgb("#f0f6fc") } else { rgb("#1f2328") }
#let bg = if dark { rgb("#1b2028") } else { white }
#let muted = if dark { rgb("#9198a1") } else { rgb("#59636e") }
#let border = if dark { rgb("#3d444d") } else { rgb("#d1d9e0") }
#let border-muted = if dark { rgb("#3d444db3") } else { rgb("#d1d9e0b3") }
#let accent = if dark { rgb("#478be6") } else { rgb("#0969da") }
#let accent2 = if dark { rgb("#94e2d5") } else { rgb("#179299") }
#let code-fg = if dark { rgb("#c9d1d9") } else { rgb("#24292f") }
#let pre-bg = if dark { rgb("#262c36") } else { rgb("#f6f8fa") }
#let shadow = if dark { rgb("#00000066") } else { rgb("#1f232826") }

#let cjk-text = regex("\p{Han}+")

#let note(body, aside) = {
    block(
        width: 100%,
        breakable: true,
        grid(
            columns: (3fr, 1fr), // body-ratio : aside-ratio
            column-gutter: 4%,
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
                text(size: fsize.tiny, aside),
            ),
        ),
    )
}

#let theorem = thmbox(
    "theorem", "定理", 
    titlefmt: strong, 
    fill: pre-bg, 
    stroke: 0.4pt + border, 
    radius: 3pt
)

#let lemma = thmbox(
    "lemma", "引理", 
    titlefmt: strong, 
    fill: pre-bg, 
    stroke: 0.4pt + border, 
    radius: 3pt
)

#let corollary = thmplain(
    "corollary", "推论", 
    titlefmt: strong
)

#let definition = thmbox(
    "definition", "定义", 
    fill: pre-bg, 
    stroke: 0.4pt + border, 
    radius: 3pt
)

#let proof-env = thmproof(
    "proof", "证明", 
    titlefmt: strong, 
    inset: (top: 0em, left: 0pt, bottom: 0em, right: 0pt)
)

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
            text(font: main-fonts, size: fsize.small, fill: muted)[
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

#let mermaid(source, width:  78%) = {

    let source = if type(source) == str {
        source
    } else if source.func() == raw {
        source.text
    } else {
        panic("mermaid expects a string or raw block")
    }

    align(
        center,
        merman(
            source,
            width: width,
            theme: (
                fontFamily: "EB Garamond,Source Han Sans",
                node_spacing: 25,
                rank_spacing: 30,
            ),
        ),
    )
}


#let template(body) = {

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
                    size: fsize.body,
                    number-type: "old-style",
                    number-width: "tabular",
                )
                place(right, dy: 3.5em)[
                    #h(1em) #text(size: 11pt, counter(page).display())
                ]
            }
        },
    )
    set columns(gutter: 4%)

    show link: set text(fill: accent2)
    show link: underline

    show strong: set text(weight: "bold")
    show emph: it => text(style: "italic", fill: accent2, it.body,)

    set par(leading: 0.86em)
    set heading(numbering: "1.1")

    show heading: it => context {
        let level = calc.min(it.level, 3)
        let size = (
            fsize.h1,
            fsize.h2,
            fsize.h3,
        ).at(level - 1)

        set text(
            font: heading-fonts,
            size: size,
            weight: "bold",
            fill: accent,
        )

        v(if level == 1 { 24pt } else { 12pt }, weak: true)

        if it.numbering != none {
            counter(heading).display(it.numbering)
            if level < 3 { h(7pt, weak: true) }
        }

        it.body

        if level == 1 {
            v(5pt, weak: true)
            line(length: 100%, stroke: 0.35pt + border-muted)
        }

        v(12pt, weak: true)
    }

    set text(
        font: main-fonts,
        fill: fg,
        weight: "regular",
        size: fsize.body,
        number-type: "old-style",
        number-width: "tabular",
    )
    show cjk-text: set text(size: 0.9em, tracking: 0.05em)

    show figure: it => block(
        above: 1.1em,
        below: 1.1em,
        it,
    )
    show figure.caption: set text(size: fsize.tiny)

    show: thmrules.with(qed-symbol: $square$)

    // code block
    show raw.where(block: true): it => {
        set text(
            font: code-fonts,
            weight: "regular",
            size: fsize.tiny,
            fill: code-fg,
        )

        v(0.8em)
        block(
            width: 100%,
            inset: 0.75em,
            radius: 3pt,
            fill: pre-bg,
            stroke: 0.4pt + border,
            it,
        )
        v(0.8em)
    }

    // inline code
    show raw.where(block: false): set text(
        font: code-fonts,
        weight: "regular",
        fill: code-fg,
    )

    // mermaid, not as a codeblock
    show raw.where(lang: "mermaid"): it => mermaid(it.text)

    // quote 
    show quote.where(block: true): it => block(
        width: 100%,
        inset: (x: 0.9em, y: 0.65em),
        fill: pre-bg,
        stroke: (left: 2pt + accent),
        radius: (right: 3pt),
        text(fill: muted)[
            #it.body

            #if it.attribution != none [
                #v(0.45em)
                #align(
                    right,
                    text(
                        size: fsize.tiny,
                        fill: muted,
                    )[
                        — #it.attribution
                    ],
                )
            ]
        ],
    )

    set table(
        inset: (x: 0.65em, y: 0.45em),
        stroke: 0.35pt + border-muted,
    )

    show table.cell.where(y: 0): it => {
        set text(
            weight: "bold",
            fill: accent,
        )
        set table.cell(fill: pre-bg)
        it
    }

    // footnote: TODO

    body
}

#let tufte(body) = template(body)
