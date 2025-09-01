# Software Development Quick Guide

**For daily use by developers at Botz Innovation**

---

## 🚀 Start Here - What Class is My Software?

```
Can software failure cause:
├─ Death or serious injury? → Class C
├─ Non-serious injury? → Class B  
└─ No injury? → Class A
```

---

## 📋 Development Checklist by Class

### Class A (Basic Safety)
- [x] Requirements doc (SWRS)
- [x] Design doc (SWDS)
- [x] Test against requirements
- [x] Basic code review
- [x] Release documentation

### Class B (Medium Safety)
Everything from Class A plus:
- [x] Detailed design required
- [x] Integration testing
- [x] Static code analysis
- [x] Unit testing
- [x] Qualified tools only

### Class C (High Safety)
Everything from Class B plus:
- [x] Extra verification rigor
- [x] Complete traceability
- [x] Formal reviews
- [x] 100% code coverage target

---

## 📝 Document Templates You Need

| Phase | Document | Template Name |
|-------|----------|---------------|
| Planning | Development Plan | Part of D&D Plan |
| Planning | Verification Plan | SWVP_Template.docx |
| Requirements | Requirements Spec | SWRS_Template.docx |
| Design | Design Spec | SWDS_Template.docx |
| Testing | Test Records | TestRecord_Template.xlsx |
| Testing | Failure Report | SWFR_Template.docx |
| Release | Release Doc | SWRE_Template.docx |

---

## 🔄 The Process Flow

```
1. PLAN
   ├─ Classify software (A/B/C)
   ├─ Create development plan
   └─ Set up tools & environment

2. SPECIFY
   ├─ Write requirements (SWRS)
   ├─ Get requirements reviewed
   └─ Each requirement gets unique ID

3. DESIGN  
   ├─ Architecture design first
   ├─ Then detailed design (SWDS)
   └─ Design review before coding!

4. CODE
   ├─ Follow coding standards
   ├─ Implement per design (no ad-hoc!)
   └─ Code review (mandatory for B/C)

5. TEST
   ├─ Unit tests (B/C only)
   ├─ Integration tests (B/C only)
   ├─ Requirements verification (all)
   └─ Document ALL failures

6. RELEASE
   ├─ Resolve or document issues
   ├─ Archive source code
   └─ Create release package
```

---

## ⚠️ Common Mistakes (Don't Do These!)

### ❌ Starting to code before design is approved
**Do instead:** Complete SWDS and get it reviewed first

### ❌ Using unqualified tools for Class B/C
**Do instead:** Check WI-XXXX for qualified tools list

### ❌ Testing without documentation
**Do instead:** Use test record template for everything

### ❌ Ignoring compiler warnings
**Do instead:** Fix them or document why they're OK

### ❌ Making design changes during coding
**Do instead:** Update SWDS first, then code

---

## 🛠️ Required Tools by Class

### Class A - Use Whatever Works
- Any IDE
- Any compiler
- Basic version control

### Class B/C - Qualified Tools Only
- **IDE**: Qualified versions only
- **Compiler**: Document version & settings
- **Static Analysis**: PC-Lint, Coverity
- **Unit Test**: Unity, Google Test
- **Version Control**: Git with documented workflow

---

## 📊 Key Metrics to Track

| Metric | Target | Class |
|--------|--------|-------|
| Requirements Coverage | 100% | All |
| Code Review Coverage | 100% | B, C |
| Unit Test Coverage | >80% | B, C |
| Static Analysis Issues | 0 Critical | B, C |
| Compiler Warnings | 0 | B, C |

---

## 🔍 Review Gates (Can't Skip These!)

### Gate 1: Requirements Review (SWRS)
- Requirements complete?
- Each one testable?
- Safety class assigned?

### Gate 2: Design Review (SWDS)
- Covers all requirements?
- Detailed enough to code from?
- Test approach defined?

### Gate 3: Code Review
- Follows standards?
- Matches design?
- No compiler warnings?

### Gate 4: Test Review
- All requirements tested?
- Failures documented?
- Ready for release?

---

## 📞 Who to Contact

| Question About | Contact |
|----------------|---------|
| Requirements unclear | Product Manager |
| Design decisions | Lead Software Developer |
| Test failures | Software Quality Engineer |
| Process questions | Quality Department |
| Tool problems | IT Support |

---

## 🚨 When to Stop and Escalate

STOP development and escalate if:
- Safety class might be wrong
- Major design change needed
- Critical bug found late
- Can't meet schedule
- Regulatory requirement unclear

---

## 📅 Time Estimates (Typical)

| Activity | Class A | Class B | Class C |
|----------|---------|---------|---------|
| Requirements | 1 week | 2 weeks | 3 weeks |
| Design | 1 week | 2 weeks | 3 weeks |
| Implementation | X weeks | X weeks | X weeks |
| Unit Testing | Optional | +20% | +30% |
| Integration Test | N/A | 1 week | 2 weeks |
| Documentation | 3 days | 1 week | 2 weeks |

---

## ✅ Release Readiness Checklist

Before declaring "Done":

- [ ] All requirements have test results
- [ ] All test failures resolved or documented
- [ ] Code reviewed and approved
- [ ] Version number assigned
- [ ] Source code archived
- [ ] Build instructions documented
- [ ] Release notes written
- [ ] Unresolved issues in USWAR
- [ ] All documents signed off

---

## 💡 Pro Tips

1. **Start documentation early** - Don't leave it to the end
2. **Use meaningful commit messages** - They're part of traceability
3. **Test as you go** - Don't accumulate technical debt
4. **Keep safety in mind** - If unsure, classify higher
5. **Communicate issues early** - Surprises at release are bad

---

**Remember: When in doubt, ask! Better to clarify than to redo work.**

---

*This is a quick guide. For complete requirements, refer to the full Software Development Procedure.*