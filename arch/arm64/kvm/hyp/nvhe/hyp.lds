SECTIONS {
 .hyp.text : { *(.text .text.*) }
 . = ALIGN((1 << 12));
 .hyp.data..percpu : {
  __per_cpu_start = .; *(.data..percpu..first) . = ALIGN((1 << 12)); *(.data..percpu..page_aligned) . = ALIGN((1 << (6))); *(.data..percpu..read_mostly) . = ALIGN((1 << (6))); *(.data..percpu) *(.data..percpu..shared_aligned) __per_cpu_end = .;
 }
}
