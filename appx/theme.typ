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
    "Noto Serif SC",
)

#let heading-fonts = (
    "Noto Sans SC",
)

#let code-fonts = (
    "Fira Code",
    "Cascadia Mono",
    "DejaVu Sans Mono",
    "Courier New",
)

#let fsize = (
  tiny: 6.5pt,
  small: 7.5pt,
  body: 8.5pt,
  h1: 12.5pt,
  h2: 10.5pt,
  h3: 9pt,
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

#let layout = sys.inputs.at("layout", default: "portrait")
#assert(
    layout in ("portrait", "landscape"),
    message: "The layout input must be either \"portrait\" or \"landscape\".",
)

#let dark = theme == "dark"
#let landscape = layout == "landscape"
#let page-columns = if landscape { 2 } else { 1 }

#let fg = if dark { rgb("#e5e7e9") } else { rgb("#26282b") }
#let bg = if dark { rgb("#18191b") } else { white }
#let muted = if dark { rgb("#989da3") } else { rgb("#62666b") }
#let border = if dark { rgb("#494e54") } else { rgb("#cbd0d5") }
#let border-muted = if dark { rgb("#494e54b3") } else { rgb("#cbd0d5b3") }
#let accent = if dark { rgb("#f0f1f2") } else { rgb("#202326") }
#let accent2 = if dark { rgb("#c0c4c8") } else { rgb("#4b5157") }
#let code-fg = if dark { rgb("#d2d5d8") } else { rgb("#33373b") }
#let pre-bg = if dark { rgb("#1e2023") } else { rgb("#f6f7f8") }
#let code-bg = if dark { rgb("#1e202399") } else { rgb("#f6f7f8cc") }
#let shadow = if dark { rgb("#00000066") } else { rgb("#383c4018") }

#let cjk-text = regex("\p{Han}+")

#let note(body, aside, side-image: none) = {
    let aside-text = if aside == [] {
        none
    } else {
        block(
            width: 100%,
            inset: (x: 0.75em, y: 0.6em),
            radius: 3pt,
            fill: pre-bg,
            text(size: fsize.tiny, aside),
        )
    }
    let aside-content = if side-image == none {
        aside-text
    } else {
        let pinned-image = if type(side-image) == str {
            image(side-image, width: 100%)
        } else {
            block(width: 100%, {
                show image: set image(width: 100%)
                side-image
            })
        }

        if aside-text == none {
            block(width: 100%, pinned-image)
        } else {
            stack(
                dir: ttb,
                spacing: 0.6em,
                block(width: 100%, pinned-image),
                aside-text,
            )
        }
    }

    block(
        width: 100%,
        breakable: true,
        grid(
            columns: (3fr, 1fr), // body-ratio : aside-ratio
            column-gutter: 4%,
            align: top + left,
            block(width: 100%, body),
            block(width: 100%, aside-content),
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

#let mermaid(source, width: 78%) = {

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
                theme-name: "base",
                background: if dark { "#18191b" } else { "#ffffff" },
                theme: (
                    fontFamily: "Noto Sans SC",
                    primaryColor: if dark { "#212326" } else { "#f6f7f8" },
                    primaryTextColor: if dark { "#e5e7e9" } else { "#26282b" },
                    primaryBorderColor: if dark { "#666b70" } else { "#b3b8bd" },
                    secondaryColor: if dark { "#1e2023" } else { "#f1f2f3" },
                    tertiaryColor: if dark { "#26282b" } else { "#eceeef" },
                    lineColor: if dark { "#989da3" } else { "#686d72" },
                    textColor: if dark { "#e5e7e9" } else { "#26282b" },
                    titleColor: if dark { "#f0f1f2" } else { "#202326" },
                    clusterBkg: if dark { "#1e2023" } else { "#fafafa" },
                    clusterBorder: if dark { "#494e54" } else { "#cbd0d5" },
                    edgeLabelBackground: if dark { "#18191b" } else { "#ffffff" },
                ),
        ),
    )
}


#let template(body) = {

    set page(
        paper: "a5",
        flipped: landscape,
        columns: page-columns,
        fill: bg,
        margin: (y: 2.25em, x: 1.8em),
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
    )
    set columns(gutter: 4%)

    show link: set text(fill: accent2)
    show link: underline

    show strong: set text(weight: "bold")
    show emph: it => text(
        weight: "medium",
        box(skew(ax: -12deg, reflow: false, it.body)),
    )

    let paragraph-leading = 0.8em
    set par(
        leading: paragraph-leading,
        spacing: 1.5 * paragraph-leading,
    )
    set heading(numbering: (..numbers) => {
        if numbers.pos().len() <= 3 {
            numbering("1.1", ..numbers)
        }
    })

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

        if it.numbering != none and it.level <= 3 {
            counter(heading).display(it.numbering)
            h(7pt, weak: true)
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
            fill: code-bg,
            it,
        )
        v(0.8em)
    }

    // inline code
    show raw.where(block: false): it => box(
        inset: (x: 0.42em, y: 0.18em),
        radius: 2.5pt,
        fill: code-bg,
        text(
            font: code-fonts,
            weight: "regular",
            fill: code-fg,
            it,
        ),
    )

    // mermaid, not as a codeblock
    show raw.where(lang: "mermaid"): it => mermaid(it.text)

    // quote 
    show quote.where(block: true): it => block(
        width: 100%,
        inset: (x: 0.9em, y: 0.65em),
        fill: pre-bg,
        stroke: (left: 1.2pt + border),
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
