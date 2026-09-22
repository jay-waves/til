
## Basic

| selectors | css | description | 
|-----------|-----|--------------|
universal | `*` | all elemnths 
element | `p` | all `<p>` elements 
class | `.box` | all elements with class `<xxx class="box">` 
id  | `#header` | all element with id `<xxx id="header">`
group | `h1, p` | all `<h1>`, `<p>` 

## Combinator 

| selectors | css | description | 
|-----------|-----|--------------|
descendant | `div p` | all `<p>` elements inside a `<div>`
child | `div > p` | direct child `<p>` elements 
adjacent sibling | `h1 + p` | `<p>` immediately after an `<h1>`
general sibling | `h1 ~ p` | all sibling `<p>` elements after an `<h1>`

## Attributes 

| selectors | css | description | 
|-----------|-----|--------------|
attibute exists | `[disabled]` | elements with the specified attribute 
exact match | `[type="text"]` | attribute equals the specified value 
contains | `[class="btn"]` | attibute contains the specfied substring 

```html
<input type="text" placeholder="Username">
<input type="password" placeholder="Password">
```

```css
input[type="text"] {
    border: 2px solid blue;
}

input[type="password"]{
    border: 2px solid red;
}
```

Examples of Attibutes CSS:
* `input[type="text"]`
* `a[href^="https://"]`
* `[aria-expanded="true"]`
* `[data-theme="dark"]`

## Pseudo-class Selectors 

Used to select elements based on their states or positions 

| selectors | css | description | 
|-----------|-----|--------------|
hover | `a:hover` |  element when hovered 
active | `button:active` | .
focus | `input:focus` | 
first child | `li:first-child` |
n-th child | `li:nth-child(2)` | 

# Priority & Cascade

Browser will considers these factors in order:
1. _Origin_ and importance `!important`
2. Cascade layers `@layer`
3. CSS Specificity 
4. Scoping proximity `@scope` 
5. Source oder (later wins)

_Origin_ referes to where a CSS rule comes from. Browser distinguishes three main origins: 
- `Author` CSS written by the wbesite developer; 
- `User` custome syltes defined by the brwoser user (user-defined css)
- `User-Agent`, default styles provided by the browser;

`Author > User > User-Agent`

## CSS Specificity 

CSS use three main categories: `(ID, Class/Attibute/Pseudo, Element)` to determine priority.
Just remember: `ID > Class > Element > Universal`

```
li {
  color: blue;          /* (0,0,1) */
}

.menu li {
  color: green;         /* (0,1,1) */
}

#header .menu li {
  color: red;           /* (1,1,1) <-- */
}
```
