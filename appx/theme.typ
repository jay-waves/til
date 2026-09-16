/// Shared document theme and typesetting helpers.
///
/// Common imports and usage (adjust the path relative to your document):
/// ```typst
/// #import "../appx/theme.typ": template, sidenote, theorem, lemma, corollary,
///   definition, proof, mermaid, fletcher, diagram, node, edge,
///   equate-lines, physica
/// #show: template
/// #set document(title: "Notes", keywords: ("robotics",))
/// ```
///
/// Feature index:
/// - `template(body)` / `template(body)`: page layout and styling; `template` is a compatibility entry point.
/// - `sidenote(body, aside, side-image: none)`: body with a right-hand sidenote and optional image.
/// - `theorem` / `lemma` / `corollary`: theorem, lemma, and corollary environments.
/// - `definition` / `proof`: definition and proof environments.
/// - `mermaid(source, width: 60%)`: Mermaid diagrams from strings or raw content.
/// - `diagram(..args)`: theme-aware Fletcher diagrams with the native API.
/// - `fletcher` / `node` / `edge`: the Fletcher module and native primitives.
/// - `equate-lines(body, ...)`: multiline equations with chapter numbers and lettered subnumbers.
/// - `physica`: physics utilities module; use `physica.xxx`.
///
/// CLI inputs: `--input theme=dark/light`, `--input layout=landscape/portrait`.
/// The recommended interfaces are listed above; other top-level names remain importable.

#import "@preview/ctheorems:1.1.3": (
    thmbox as _thm_box,
    thmplain as _thm_plain,
    thmproof as _thm_proof,
    thmrules as _thm_rules,
)
#import "@preview/physica:0.9.8" as physica
#import "@preview/merman:0.1.0": mermaid as _merman
#import "@preview/fletcher:0.5.8" as fletcher
#import "@preview/equate:0.3.3": equate as _equate

#assert(
    sys.version >= version(0, 15, 0),
    message: "This theme requires Typst 0.15.0 or newer.",
)

#let _main_fonts = (
    "Noto Serif SC",
)

#let _heading_fonts = (
    "Noto Sans SC",
)

#let _code_fonts = (
    "Fira Code",
    "Cascadia Mono",
    "DejaVu Sans Mono",
    "Courier New",
)

#let _font_size = (
  tiny: 6.5pt,
  small: 7.5pt,
  body: 8.5pt,
  h1: 12.5pt,
  h2: 10.5pt,
  h3: 9pt,
)

#let _spacing = (
    xs: 5pt,
    sm: 8pt,
    md: 12pt,
    lg: 18pt,
    xl: 32pt,
)

#let _preview_info = json(
    bytes(sys.inputs.at("x-preview", default: "{}"))
)

#let _theme = sys.inputs.at(
    "theme",
    default: _preview_info.at("theme", default: "light"),
)

#let _layout = sys.inputs.at("layout", default: "portrait")
#assert(
    _layout in ("portrait", "landscape"),
    message: "The layout input must be either \"portrait\" or \"landscape\".",
)

#let _dark = _theme == "dark"
#let _landscape = _layout == "landscape"
#let _page_columns = if _landscape { 2 } else { 1 }

#let _fg = if _dark { rgb("#c9cdd2") } else { rgb("#26282b") }
#let _bg = if _dark { rgb("#1e1e1e") } else { white }
#let _muted = if _dark { rgb("#a0a5ad") } else { rgb("#62666b") }
#let _border = if _dark { rgb("#50555d") } else { rgb("#cbd0d5") }
#let _border_muted = if _dark { rgb("#50555db3") } else { rgb("#cbd0d5b3") }
#let _accent = if _dark { rgb("#dde0e4") } else { rgb("#202326") }
#let _accent_2 = if _dark { rgb("#bfc5cd") } else { rgb("#4b5157") }
#let _code_fg = if _dark { rgb("#c3c9d1") } else { rgb("#33373b") }
#let _pre_bg = if _dark { rgb("#2d3035") } else { rgb("#f6f7f8") }
#let _code_bg = if _dark { rgb("#2d3035cc") } else { rgb("#f6f7f8cc") }
#let _shadow = if _dark { rgb("#00000066") } else { rgb("#383c4018") }

#let _cjk_text = regex("\p{Han}+")

// Re-export Fletcher's node and edge primitives unchanged.
#let node = fletcher.node
#let edge = fletcher.edge

/// Fletcher's native diagram interface with theme-aware defaults.
/// Explicit call-site options override the injected defaults.
#let diagram(..args) = align(center, {
    set text(
        font: _main_fonts,
        size: _font_size.tiny,
        fill: _fg,
    )

    fletcher.diagram.with(
        spacing: 3em,
        node-fill: _pre_bg,
        node-stroke: 0.5pt + _border,
        edge-stroke: 0.55pt + _muted,
    )(..args)
})

/// Body with a right-hand sidenote: `#sidenote[Body][Aside]`; `side-image` accepts an image path or content.
#let sidenote(body, aside, side-image: none) = {
    let aside-text = if aside == [] {
        none
    } else {
        block(
            width: 100%,
            inset: (x: 0.75em, y: 0.6em),
            radius: 3pt,
            fill: _pre_bg,
            text(size: _font_size.tiny, aside),
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
#let theorem = _thm_box(
    "theorem", "定理", 
    supplement: [Thm.],
    titlefmt: strong, 
    fill: _pre_bg, 
    stroke: 0.4pt + _border, 
    radius: 3pt
)

// Titleless, unnumbered theorem box used to render block quotes.
#let _quote_box = _thm_box(
    "quote", [],
    supplement: none,
    titlefmt: body => body,
    separator: [],
    fill: _pre_bg,
    stroke: 0.4pt + _border,
    radius: 3pt,
).with(numbering: none)

/// Lemma environment: `#lemma[Lemma content]`.
#let lemma = _thm_box(
    "lemma", "引理", 
    supplement: [Lemma],
    titlefmt: strong, 
    fill: _pre_bg, 
    stroke: 0.4pt + _border, 
    radius: 3pt
)

/// Corollary environment: `#corollary[Corollary content]`.
#let corollary = _thm_plain(
    "corollary", "推论", 
    supplement: [Cor.],
    titlefmt: strong
)

/// Definition environment: `#definition[Definition content]`.
#let definition = _thm_box(
    "definition", "定义", 
    supplement: [Def.],
    fill: _pre_bg, 
    stroke: 0.4pt + _border, 
    radius: 3pt
)

#let _proof_env = _thm_proof(
    "proof", "证明", 
    titlefmt: strong, 
    inset: (top: 0em, left: 0pt, bottom: 0em, right: 0pt)
)

/// Proof environment: `#proof[Proof content]`; automatically adds a QED symbol.
#let proof(..args, body) = {
    _proof_env(..args, body)
    linebreak()
}

/// Mermaid's native interface with theme-aware defaults; `width` defaults to `60%`.
#let mermaid(..args) = {
    align(
        center,
        _merman.with(
                width: 60%,
                theme-name: "base",
                background: _bg.to-hex(),
                theme: (
                    fontFamily: _main_fonts.first(),
                    primaryColor: _pre_bg.to-hex(),
                    primaryTextColor: _fg.to-hex(),
                    primaryBorderColor: if _dark { "#707680" } else { "#b3b8bd" },
                    secondaryColor: if _dark { _pre_bg.to-hex() } else { "#f1f2f3" },
                    tertiaryColor: if _dark { "#353940" } else { "#eceeef" },
                    lineColor: if _dark { _muted.to-hex() } else { "#686d72" },
                    textColor: _fg.to-hex(),
                    titleColor: _accent.to-hex(),
                    clusterBkg: if _dark { _pre_bg.to-hex() } else { "#fafafa" },
                    clusterBorder: _border.to-hex(),
                    edgeLabelBackground: _bg.to-hex(),
                ),
        )(..args),
    )
}


// Chapter-prefixed numbering shared by figures, tables, and equations.
#let _chapter_numbering(parenthesized, styled, number, ..sub) = context {
    let chapter = counter(heading).get().first()
    let suffix = if sub.pos().len() > 0 { numbering("a", sub.pos().first()) } else { "" }
    let value = str(chapter) + "." + str(number) + suffix
    let value = if parenthesized { "(" + value + ")" } else { value }

    if not styled {
        value
    } else {
    // Keep serif letterforms while reserving equal space for each sub-number letter.
        show regex("[a-z]+"): it => {
            it.text.clusters().map(letter => box(width: 0.55em, align(center, letter))).join()
        }
        text(
            font: _main_fonts,
            size: _font_size.tiny,
            fill: _muted,
            number-type: "lining",
            number-width: "tabular",
            value,
        )
    }
}

#let _equation_numbering = _chapter_numbering.with(true, true)
#let _figure_numbering = _chapter_numbering.with(false, false)

/// Multiline equations with chapter numbers and lettered subnumbers; disable with `sub-numbering: false`.
/// Label each line with `#<label>` before its line break.
#let equate-lines(
    body,
    numbering: _equation_numbering,
    sub-numbering: true,
    ..options,
) = {
    set math.equation(numbering: if numbering == auto { _equation_numbering } else { numbering })
    _equate(body, sub-numbering: sub-numbering, ..options)
}

/// Main layout entry point: `#show: template`; accepts `theme` and `layout` CLI inputs.
#let template(body) = {

    set math.equation(numbering: _equation_numbering, supplement: [Eq.])
    set heading(supplement: [Sec.])
    show figure.where(kind: image): set figure(supplement: [Fig.], numbering: _figure_numbering)
    show figure.where(kind: table): set figure(supplement: [Tab.], numbering: _figure_numbering)
    show ref: it => context {
        // Resolve the chapter at the target, including equate's per-line figures.
        let target = it.element
        if it.form == "normal" and target != none and target.has("numbering") and target.numbering == _equation_numbering {
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
            _equate(it)
        } else {
            _equate(it)
        }
    }

    set page(
        paper: "a5",
        flipped: _landscape,
        columns: _page_columns,
        fill: _bg,
        margin: (y: 2.25em, x: 1.8em),
        header: counter(footnote).update(0),
        foreground: if _landscape {
            place(
                center + horizon,
                rect(
                    width: 0.55pt,
                    height: 100% - 6em,
                    fill: _border_muted,
                ),
            )
        },
    )
    set columns(gutter: 4%)

    show link: set text(fill: _accent_2)
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
            _font_size.h1,
            _font_size.h2,
            _font_size.h3,
        ).at(level - 1)

        set text(
            font: _heading_fonts,
            size: size,
            weight: "bold",
            fill: _accent,
        )

        v(if level == 1 { 24pt } else { 12pt }, weak: true)

        if it.numbering != none and it.level <= 3 {
            counter(heading).display(it.numbering)
            h(7pt, weak: true)
        }

        it.body

        if level == 1 {
            v(5pt, weak: true)
            line(length: 100%, stroke: 0.35pt + _border_muted)
        }

        v(12pt, weak: true)
    }

    set text(
        font: _main_fonts,
        fill: _fg,
        weight: "regular",
        size: _font_size.body,
        number-type: "old-style",
        number-width: "tabular",
    )
    show figure: fig => {
        show figure.caption: caption => context [
            *#caption.supplement~#numbering(
                caption.numbering,
                ..caption.counter.at(fig.location()),
            )#h(1em)*#caption.body
        ]
        block(
            above: 1.1em,
            below: 1.1em,
            fig,
        )
    }
    show figure.caption: set text(size: _font_size.tiny)

    show: _thm_rules.with(qed-symbol: $square$)

    // code block
    show raw.where(block: true): it => {
        set text(
            font: _code_fonts,
            weight: "regular",
            size: _font_size.tiny,
            fill: _code_fg,
        )

        v(0.8em)
        block(
            width: 100%,
            inset: 0.75em,
            radius: 3pt,
            fill: _code_bg,
            it,
        )
        v(0.8em)
    }

    // inline code
    show raw.where(block: false): it => box(
        inset: (x: 0.42em, y: 0.18em),
        radius: 2.5pt,
        fill: _code_bg,
        text(
            font: _code_fonts,
            weight: "regular",
            fill: _code_fg,
            it,
        ),
    )

    // Render block quotes as titleless, unnumbered ctheorems boxes.
    show quote: it => _quote_box[
        #text(fill: _muted)[
            #it.body

            #if it.attribution != none [
                #v(0.45em)
                #align(
                    right,
                    text(
                        size: _font_size.tiny,
                        fill: _muted,
                    )[
                        — #it.attribution
                    ],
                )
            ]
        ]
    ]

    set table(
        inset: (x: 0.65em, y: 0.45em),
        stroke: 0.35pt + _border_muted,
    )

    show table.cell.where(y: 0): it => {
        set text(
            weight: "bold",
            fill: _accent,
        )
        set table.cell(fill: _pre_bg)
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
