/// Shared document theme and typesetting helpers.
///
/// Common imports and usage (adjust the path relative to your document):
/// ```typst
/// #import "../appx/theme.typ": template, note, theorem, lemma, corollary,
///   definition, proof, mermaid, equate-lines, physica
/// #show: template
/// #set document(title: "Notes", keywords: ("robotics",))
/// ```
///
/// Feature index:
/// - `template(body)` / `tufte(body)`: page layout and styling; `tufte` is a compatibility entry point.
/// - `note(body, aside, side-image: none)`: body with a right-hand sidenote and optional image.
/// - `theorem` / `lemma` / `corollary`: theorem, lemma, and corollary environments.
/// - `definition` / `proof`: definition and proof environments.
/// - `mermaid(source, width: 78%)`: Mermaid diagrams from strings or raw content.
/// - `equate-lines(body, ...)`: multiline equations with chapter numbers and lettered subnumbers.
/// - `physica`: physics utilities module; use `physica.xxx`.
///
/// Style variables: `main-fonts`, `heading-fonts`, `code-fonts`, `fsize`, `spacing`,
/// `fg`, `bg`, `muted`, `accent`, `accent2`, `border`, `border-muted`, `code-fg`,
/// `pre-bg`, `code-bg`, and `shadow`.
///
/// CLI inputs: `--input theme=dark/light`, `--input layout=landscape/portrait`.
/// The recommended interfaces are listed above; other top-level names remain importable.

#import "@preview/ctheorems:1.1.3": *
#import "@preview/physica:0.9.8" as physica
#import "@preview/merman:0.1.0": mermaid as merman
#import "@preview/equate:0.3.3": equate

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

#let fg = if dark { rgb("#c9cdd2") } else { rgb("#26282b") }
#let bg = if dark { rgb("#1e1e1e") } else { white }
#let muted = if dark { rgb("#a0a5ad") } else { rgb("#62666b") }
#let border = if dark { rgb("#50555d") } else { rgb("#cbd0d5") }
#let border-muted = if dark { rgb("#50555db3") } else { rgb("#cbd0d5b3") }
#let accent = if dark { rgb("#dde0e4") } else { rgb("#202326") }
#let accent2 = if dark { rgb("#bfc5cd") } else { rgb("#4b5157") }
#let code-fg = if dark { rgb("#c3c9d1") } else { rgb("#33373b") }
#let pre-bg = if dark { rgb("#2d3035") } else { rgb("#f6f7f8") }
#let code-bg = if dark { rgb("#2d3035cc") } else { rgb("#f6f7f8cc") }
#let shadow = if dark { rgb("#00000066") } else { rgb("#383c4018") }

#let cjk-text = regex("\p{Han}+")

/// Body with a right-hand sidenote: `#note[Body][Aside]`; `side-image` accepts an image path or content.
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

/// Theorem environment: `#theorem[Theorem content]`.
#let theorem = thmbox(
    "theorem", "定理", 
    supplement: [Thm.],
    titlefmt: strong, 
    fill: pre-bg, 
    stroke: 0.4pt + border, 
    radius: 3pt
)

/// Lemma environment: `#lemma[Lemma content]`.
#let lemma = thmbox(
    "lemma", "引理", 
    supplement: [Lemma],
    titlefmt: strong, 
    fill: pre-bg, 
    stroke: 0.4pt + border, 
    radius: 3pt
)

/// Corollary environment: `#corollary[Corollary content]`.
#let corollary = thmplain(
    "corollary", "推论", 
    supplement: [Cor.],
    titlefmt: strong
)

/// Definition environment: `#definition[Definition content]`.
#let definition = thmbox(
    "definition", "定义", 
    supplement: [Def.],
    fill: pre-bg, 
    stroke: 0.4pt + border, 
    radius: 3pt
)

#let proof-env = thmproof(
    "proof", "证明", 
    titlefmt: strong, 
    inset: (top: 0em, left: 0pt, bottom: 0em, right: 0pt)
)

/// Proof environment: `#proof[Proof content]`; automatically adds a QED symbol.
#let proof(..args, body) = {
    proof-env(..args, body)
    linebreak()
}

/// Render a Mermaid diagram from a string or raw content; `width` defaults to `78%`.
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
                background: bg.to-hex(),
                theme: (
                    fontFamily: "Noto Sans SC",
                    primaryColor: pre-bg.to-hex(),
                    primaryTextColor: fg.to-hex(),
                    primaryBorderColor: if dark { "#707680" } else { "#b3b8bd" },
                    secondaryColor: if dark { pre-bg.to-hex() } else { "#f1f2f3" },
                    tertiaryColor: if dark { "#353940" } else { "#eceeef" },
                    lineColor: if dark { muted.to-hex() } else { "#686d72" },
                    textColor: fg.to-hex(),
                    titleColor: accent.to-hex(),
                    clusterBkg: if dark { pre-bg.to-hex() } else { "#fafafa" },
                    clusterBorder: border.to-hex(),
                    edgeLabelBackground: bg.to-hex(),
                ),
        ),
    )
}


// Internal numbering helper shared by ordinary equations and equate-lines.
#let equation-numbering(number, ..sub) = context {
    let chapter = counter(heading).get().first()
    let suffix = if sub.pos().len() > 0 { numbering("a", sub.pos().first()) } else { "" }
    // Keep serif letterforms while reserving equal space for each sub-number letter.
    show regex("[a-z]+"): it => {
        it.text.clusters().map(letter => box(width: 0.55em, align(center, letter))).join()
    }
    text(
        font: main-fonts,
        size: fsize.tiny,
        fill: muted,
        number-type: "lining",
        number-width: "tabular",
        "(" + str(chapter) + "." + str(number) + suffix + ")",
    )
}

/// Multiline equations with chapter numbers and lettered subnumbers; disable with `sub-numbering: false`.
/// Label each line with `#<label>` before its line break.
#let equate-lines(
    body,
    numbering: equation-numbering,
    sub-numbering: true,
    ..options,
) = {
    set math.equation(numbering: if numbering == auto { equation-numbering } else { numbering })
    equate(body, sub-numbering: sub-numbering, ..options)
}

/// Main layout entry point: `#show: template`; accepts `theme` and `layout` CLI inputs.
#let template(body) = {

    set math.equation(numbering: equation-numbering, supplement: [Eq.])
    set heading(supplement: [Sec.])
    show figure.where(kind: image): set figure(supplement: [Fig.])
    show figure.where(kind: table): set figure(supplement: [Table])
    show ref: it => context {
        // Resolve the chapter at the target, including equate's per-line figures.
        let target = it.element
        if it.form == "normal" and target != none and target.has("numbering") and target.numbering == equation-numbering {
            let chapter = counter(heading).at(target.location()).first()
            // The numbering function also runs inside refs; restore the surrounding text style.
            let ref-size = text.size
            let ref-font = text.font
            let ref-fill = text.fill
            show regex("\\([0-9]+\\.[0-9]+[a-z]*\\)"): match => {
                text(
                    font: ref-font,
                    size: ref-size,
                    fill: ref-fill,
                    str(chapter) + "." + match.text.slice(1, -1).split(".").last(),
                )
            }
            equate(it)
        } else {
            equate(it)
        }
    }

    set page(
        paper: "a5",
        flipped: landscape,
        columns: page-columns,
        fill: bg,
        margin: (y: 2.25em, x: 1.8em),
        header: counter(footnote).update(0),
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

    // Raise regular body text (400) to bold (700).
    set strong(delta: 300)
    show emph: it => text(
        weight: 550,
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
        if it.level == 1 and it.numbering != none {
            counter(math.equation).update(0)
        }
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

    // Footnotes restart on each page: superscript [a]; entries: [a] content.
    set footnote(numbering: "[a]")
    show footnote.entry: it => context {
        let note = it.note
        let number = counter(footnote).at(note.location()).first()
        block[
            #link(note.location(), numbering(note.numbering, number))#h(0.3em)#note.body
        ]
    }

    body
}

/// Compatibility entry point for `template`; prefer `#show: template` in new documents.
#let tufte(body) = template(body)
