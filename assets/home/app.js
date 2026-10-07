(() => {
  const $ = s => document.querySelector(s);
  const esc = s => String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  let lang = new URLSearchParams(location.search).get('lang');
  if (!['en','zh'].includes(lang)) { try { lang=localStorage.getItem('portfolio-language'); } catch {} }
  if (!['en','zh'].includes(lang)) lang='en';
  const drawings={
    exo:'<path d="M250 25L230 78L276 110L255 157" stroke-width="22"/><path d="M277 23L255 77L299 109L281 157" stroke-width="4"/><circle cx="263" cy="99" r="20" fill="#e4e9db" stroke-width="4"/><circle cx="263" cy="99" r="8"/><path d="M192 62H224M302 124H335" stroke-dasharray="3 4"/><circle cx="192" cy="62" r="4"/><circle cx="335" cy="124" r="4"/>',
    arm:'<path d="M160 153H315M230 151V112L306 76L263 30L325 23" stroke-width="12"/><circle cx="230" cy="112" r="13" fill="#e2e7e8" stroke-width="3"/><circle cx="306" cy="76" r="13" fill="#e2e7e8" stroke-width="3"/><path d="M160 103C183 16 350 152 365 51" stroke-width="2" stroke-dasharray="4 5"/>',
    radar:'<circle cx="258" cy="94" r="18" fill="#738765"/><circle cx="258" cy="94" r="42"/><circle cx="258" cy="94" r="67"/><circle cx="258" cy="94" r="91" stroke-dasharray="3 5"/><path d="M258 94L319 49" stroke-width="3"/><circle cx="313" cy="54" r="6" fill="#e1b983"/>',
    chip:'<rect x="214" y="43" width="92" height="92" rx="8" stroke-width="3"/><rect x="237" y="65" width="47" height="47" rx="3"/><path d="M225 22V43M243 22V43M261 22V43M280 22V43M297 22V43M225 135V157M243 135V157M261 135V157M280 135V157M297 135V157M192 55H214M192 77H214M192 99H214M192 122H214M306 55H328M306 77H328M306 99H328M306 122H328" stroke-width="3"/>'
  };
  const tags = a => a.map(x=>`<span>${esc(x)}</span>`).join('');
  const timeline = a => a.map(x=>`<article class="timeline-item"><span class="date">${esc(x.date)}</span><h3>${esc(x.title)}</h3><h4>${esc(x.subtitle)}</h4><p>${esc(x.text)}</p></article>`).join('');
  function photo(m,cover){
    if(m.video){
      return `<div class="video-wrap ${cover?'cover':''}"><video src="assets/home/photos/${esc(m.video)}" autoplay muted loop playsinline preload="metadata" aria-label="${esc(m[lang])}"></video></div>`;
    }
    return `<button class="photo-button ${cover?'cover':'thumbnail'}" data-image="${esc(m.file)}" data-caption="${esc(m[lang])}" aria-label="${lang==='zh'?'放大图片：':'Enlarge image: '}${esc(m[lang])}"><img src="assets/home/photos/${esc(m.file)}" alt="${esc(m[lang])}" loading="lazy" decoding="async"></button>`;
  }
  let revealDone = false;
  function setupReveal(){
    if(revealDone || matchMedia('(prefers-reduced-motion: reduce)').matches){
      document.querySelectorAll('.reveal').forEach(el=>el.classList.add('in'));
      revealDone = true; return;
    }
    const io = new IntersectionObserver(entries=>{
      entries.forEach(en=>{ if(en.isIntersecting){ en.target.classList.add('in'); io.unobserve(en.target); } });
    },{threshold:.1,rootMargin:'0px 0px -40px 0px'});
    document.querySelectorAll('.reveal:not(.in)').forEach(el=>io.observe(el));
    revealDone = true;
  }
  function render(){
    const d=PROFILE[lang];document.documentElement.lang=lang;
    document.querySelectorAll('[data-t]').forEach(el=>{el.textContent=d[el.dataset.t]||'';});
    $('#language').textContent=lang==='en'?'中文':'EN';$('#language').setAttribute('aria-label',lang==='en'?'切换到中文':'Switch to English');
    if($('#interests')) $('#interests').innerHTML=tags(d.interests);
    if($('#skills')) $('#skills').innerHTML=tags(PROFILE.skills);
    if($('#project-grid')) $('#project-grid').innerHTML=d.projects.map(p=>{
      const media=PROFILE.media[p.art]||[];
      const cover=media.length ? `<div class="project-art project-photo">${photo(media[0],true)}<span class="photo-type">${esc(p.type)}</span></div><p class="photo-caption">${esc(media[0][lang])}</p>` : `<div class="project-art" aria-hidden="true"><span class="label">${esc(p.type)}</span><svg viewBox="0 0 520 180" fill="none" stroke="#536e5c" stroke-width="1.3" stroke-linecap="round" stroke-linejoin="round">${drawings[p.art]}</svg><span class="number">${p.id}</span></div><p class="photo-caption">${lang==='zh'?'感知原理示意图':'Sensing concept illustration'}</p>`;
      const gallery=media.slice(1).map(m=>`<figure class="${m.video?'has-video':''}">${photo(m,false)}<figcaption>${esc(m[lang])}</figcaption></figure>`).join('');
      const videoNote=media.some(m=>m.video)?`<span class="video-note">${esc(d.videoLabel)} · ${esc(d.videoLabelHint)}</span>`:'';
      return `<article class="project reveal">${cover}<div class="project-body"><span class="project-meta">${esc(p.date)}</span><h3>${esc(p.title)}</h3><p>${esc(p.summary)}</p><div class="tags">${tags(p.tags)}</div><details><summary>${esc(d.details)}</summary><ul>${p.details.map(x=>`<li>${esc(x)}</li>`).join('')}</ul>${videoNote}${gallery?`<div class="project-gallery">${gallery}</div>`:''}</details></div></article>`;
    }).join('');
    if($('#experience-list')) $('#experience-list').innerHTML=timeline(d.experience);
    if($('#education-list')) $('#education-list').innerHTML=timeline(d.education);
    if($('#awards')) $('#awards').innerHTML=d.awards.map(x=>`<li>${esc(x)}</li>`).join('');
    if($('#email-link')){$('#email-link').href='mailto:'+PROFILE.email;$('#email-link').textContent=PROFILE.email+' ↗';}
    if($('#cv-content')){
      $('#cv-content').innerHTML=`<div class="cv-name"><h1>Boning Tan <span>谭泊宁</span></h1><p>${esc(d.role)}</p><p><a href="mailto:${esc(PROFILE.email)}">${esc(PROFILE.email)}</a> · <a href="https://tanboning1118.github.io/">tanboning1118.github.io</a> · <a href="https://github.com/tanboning1118">GitHub</a></p></div><h2>${esc(d.educationLabel)}</h2>${timeline(d.education)}<h2>${esc(d.experienceLabel)}</h2>${timeline(d.experience)}<h2>${esc(d.publication)}</h2><article><p>Guang Chen, Binyao Zhang, <strong>Boning Tan</strong>, Weizhong Yuan. Electric Eel-Inspired Elastomer-Encapsulated Hydrogel Sensor with Superior Anti-Swelling, Self-Adhesion, and Electrical Stability for Diverse Underwater Applications. <em>Small</em>, 21(34), 2025. <a href="https://doi.org/10.1002/smll.202503207">doi:10.1002/smll.202503207</a></p><p>${esc(d.paperContribution)}</p></article><h2>${esc(d.projectsHeading)}</h2>${d.projects.map(p=>`<article class="cv-project"><div class="cv-project-title"><h3>${esc(p.title)}</h3><span>${esc(p.date)}</span></div><ul>${p.details.map(x=>`<li>${esc(x)}</li>`).join('')}</ul></article>`).join('')}<h2>${esc(d.awardsLabel)}</h2><ul>${d.awards.map(x=>`<li>${esc(x)}</li>`).join('')}</ul><h2>${esc(d.toolkit)}</h2><p>${PROFILE.skills.map(esc).join(' · ')}</p><p class="cv-updated">${lang==='en'?'Updated':'更新'}: ${PROFILE.updated}</p>`;
    }
    document.querySelectorAll('a[href^="cv.html"]').forEach(a=>a.href='cv.html?lang='+lang);
    document.querySelectorAll('[data-pdf]').forEach(a=>{a.href=`assets/home/Boning-Tan-CV-${lang}.pdf`;a.textContent=lang==='en'?'Download PDF ↓':'下载 PDF ↓';});
    if($('#hero-photo-caption')){
      $('#hero-photo-caption').textContent=PROFILE.media.hero[lang];
      const hero=$('.hero-image-button');hero.dataset.caption=PROFILE.media.hero[lang];hero.setAttribute('aria-label',(lang==='zh'?'放大图片：':'Enlarge image: ')+PROFILE.media.hero[lang]);hero.querySelector('img').alt=PROFILE.media.hero[lang];
      $('.viewer-close').setAttribute('aria-label',lang==='zh'?'关闭图片':'Close image');
    }
    if($('#year'))$('#year').textContent=new Date().getFullYear();
    setupReveal();
  }
  $('#language').addEventListener('click',()=>{lang=lang==='en'?'zh':'en';try{localStorage.setItem('portfolio-language',lang);}catch{}const u=new URL(location);u.searchParams.set('lang',lang);history.replaceState(null,'',u);render();});
  if($('#print'))$('#print').addEventListener('click',()=>window.print());
  const viewer=$('#image-viewer');
  if(viewer){
    document.addEventListener('click',e=>{const trigger=e.target.closest('[data-image]');if(!trigger)return;$('#viewer-image').src='assets/home/photos/'+trigger.dataset.image;$('#viewer-image').alt=trigger.dataset.caption;$('#image-caption').textContent=trigger.dataset.caption;viewer.showModal();document.body.classList.add('viewing-image');});
    $('.viewer-close').addEventListener('click',()=>viewer.close());
    viewer.addEventListener('click',e=>{if(e.target===viewer)viewer.close();});
    viewer.addEventListener('close',()=>document.body.classList.remove('viewing-image'));
  }
  const header=$('.header');
  if(header){ addEventListener('scroll',()=>header.classList.toggle('scrolled',scrollY>10),{passive:true}); }
  render();
})();
